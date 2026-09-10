"""
single_drone_control.py

SINGLE-DRONE control script — roadmap step 4 ("get single-drone control
rock solid" before moving to multi-drone coordination).

Scope of this file, on purpose:
    connect -> arm -> takeoff -> fly to one GPS waypoint -> hold -> RTL -> land
    + continuous telemetry read-back the whole time.

This is NOT the swarm coordinator. Multi-drone logic (sector assignment,
running 4 of these at once, battery-triggered auto-RTL across the fleet)
belongs in control/multi_drone_control.py later (roadmap step 6) — don't
grow this file into that. Keep this one boring and reliable; every bug
fixed here would otherwise get multiplied by 4 later.

Commands used below map directly to docs/protocol.md's command table:
    ARM, TAKEOFF, GOTO, HOLD, RTL
Telemetry fields printed below map to docs/protocol.md's telemetry table.
Fields that only make sense at the swarm level (assigned_task, kit_status,
rc_override_active) are noted but not implemented here — they belong in
the multi-drone version once there's an actual base-station process to
send/receive them.

Requires: mavsdk (see requirements.txt)

Run against ArduPilot SITL:
    1. Start SITL in a separate terminal:
         sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14540
    2. Run this script:
         python control/single_drone_control.py

Run against a real Pixhawk (once step 4 is solid in sim):
    python control/single_drone_control.py --connection serial:///dev/ttyACM0:57600
"""

import argparse
import asyncio
import logging
from datetime import datetime
from pathlib import Path

from mavsdk import System
from mavsdk.action import ActionError

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("single_drone")

# Default target altitude for takeoff and the test waypoint (meters, relative to home)
DEFAULT_TAKEOFF_ALT_M = 10.0
# Default test waypoint offset — replace with a real field waypoint once
# docs/mission_spec.md has actual venue GPS bounds instead of TODO (venue)
DEFAULT_WAYPOINT_LAT = 47.397606
DEFAULT_WAYPOINT_LON = 8.543060
DEFAULT_WAYPOINT_ALT_M = 10.0

# ADDED: timeouts so the script never hangs forever if SITL / vehicle
# misbehaves. Values are generous for SITL; tighten once you know real
# hardware timings. A hang during the 30-min competition window is a dead
# mission, so every blocking wait must be bounded.
TIMEOUT_CONNECT_S = 30.0
TIMEOUT_HEALTH_S = 60.0
TIMEOUT_TAKEOFF_S = 60.0
TIMEOUT_GOTO_S = 180.0
TIMEOUT_LAND_S = 120.0


# ADDED: file logging so field failures have a durable record. Terminal
# scrollback is not enough at the field.
def _setup_file_logging() -> None:
    log_dir = Path("logs")
    log_dir.mkdir(exist_ok=True)
    fname = log_dir / f"single_drone_{datetime.now():%Y%m%d_%H%M%S}.log"
    handler = logging.FileHandler(fname)
    handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
    logging.getLogger().addHandler(handler)
    log.info("Logging to %s", fname)


async def connect_drone(connection_string: str) -> System:
    """ARM/TAKEOFF/GOTO all require a connected System first.

    FIX: both waits are now bounded by asyncio.wait_for — previously this
    hung forever if SITL wasn't running, the connection string was wrong,
    or GPS never locked. Also now waits for is_armable (EKF / battery /
    compass pre-arm checks), not just global+home position.
    """
    drone = System()
    log.info("Connecting to drone on %s ...", connection_string)
    await drone.connect(system_address=connection_string)

    async def _wait_connected():
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    try:
        await asyncio.wait_for(_wait_connected(), timeout=TIMEOUT_CONNECT_S)
        log.info("Drone connected.")
    except asyncio.TimeoutError:
        raise RuntimeError(
            f"Timed out after {TIMEOUT_CONNECT_S}s waiting for connection. "
            "Is SITL running? Is the connection string correct?"
        )

    log.info("Waiting for pre-arm health checks (global pos, home pos, is_armable)...")

    async def _wait_health():
        async for health in drone.telemetry.health():
            if (health.is_armable
                    and health.is_global_position_ok
                    and health.is_home_position_ok):
                return health

    try:
        health = await asyncio.wait_for(_wait_health(), timeout=TIMEOUT_HEALTH_S)
    except asyncio.TimeoutError:
        # Print the individual flags so the user knows which check is blocking.
        async for h in drone.telemetry.health():
            log.error(
                "Health timeout — is_armable=%s global_pos_ok=%s home_pos_ok=%s "
                "gyro_ok=%s accel_ok=%s mag_ok=%s local_pos_ok=%s",
                h.is_armable, h.is_global_position_ok, h.is_home_position_ok,
                h.is_gyrometer_calibration_ok, h.is_accelerometer_calibration_ok,
                h.is_magnetometer_calibration_ok, h.is_local_position_ok,
            )
            break
        raise RuntimeError(f"Timed out after {TIMEOUT_HEALTH_S}s waiting for pre-arm health.")

    log.info("Pre-arm health OK.")
    return drone


# ADDED: fetch home altitude once. Needed because MAVSDK's goto_location
# takes AMSL altitude, not relative-to-home. See goto_waypoint for why.
async def get_home_altitude(drone: System) -> float:
    async def _wait_home():
        async for home in drone.telemetry.home():
            return home.absolute_altitude_m

    try:
        home_alt = await asyncio.wait_for(_wait_home(), timeout=TIMEOUT_HEALTH_S)
        log.info("Home altitude (AMSL): %.2f m", home_alt)
        return home_alt
    except asyncio.TimeoutError:
        raise RuntimeError("Timed out waiting for home altitude.")


async def monitor_telemetry(drone: System):
    """Continuous telemetry read-back — runs concurrently with the mission
    steps below via asyncio.create_task(). Field names/comments map to
    docs/protocol.md's telemetry table.
    """
    async def watch_position():
        async for position in drone.telemetry.position():
            # -> protocol.md: gps { lat, lon, alt }
            # NOTE: absolute_altitude_m is AMSL (what goto_location wants);
            # relative_altitude_m is altitude above home (what takeoff uses).
            # Printing both makes the AMSL-vs-relative distinction obvious.
            log.info(
                "TELEMETRY gps: lat=%.7f lon=%.7f alt_rel=%.2fm alt_amsl=%.2fm",
                position.latitude_deg, position.longitude_deg,
                position.relative_altitude_m, position.absolute_altitude_m,
            )

    async def watch_battery():
        async for battery in drone.telemetry.battery():
            # -> protocol.md: battery_pct
            log.info("TELEMETRY battery_pct: %.0f%%", battery.remaining_percent * 100)

    async def watch_flight_mode():
        async for mode in drone.telemetry.flight_mode():
            # -> protocol.md: mode
            log.info("TELEMETRY mode: %s", mode)

    async def watch_armed():
        async for armed in drone.telemetry.armed():
            # -> protocol.md: state (approximated here; full state machine
            # from protocol.md — IDLE/ARMING/TAKEOFF/SEARCHING/etc. — is a
            # multi-drone-version concern once there's mission logic to
            # drive those transitions)
            log.info("TELEMETRY armed: %s", armed)

    # NOTE: rc_override_active, health (full struct), assigned_task, and
    # kit_status from protocol.md are not populated here on purpose —
    # they either need swarm-level context (assigned_task, kit_status)
    # or RC-override detection wiring (rc_override_active) that belongs
    # in the multi-drone version, not this single-drone baseline.

    await asyncio.gather(
        watch_position(),
        watch_battery(),
        watch_flight_mode(),
        watch_armed(),
    )


async def arm_and_takeoff(drone: System, altitude_m: float):
    """Maps to protocol.md commands: ARM, then TAKEOFF { altitude }."""
    log.info("ARM")
    try:
        await drone.action.arm()
    except ActionError as e:
        log.error("Arm failed: %s", e)
        raise

    log.info("TAKEOFF altitude=%.1fm", altitude_m)
    # NOTE: set_takeoff_altitude IS relative to ground — correct as-is.
    await drone.action.set_takeoff_altitude(altitude_m)
    await drone.action.takeoff()

    # FIX: bounded wait. Previously hung forever if takeoff silently failed.
    async def _wait_altitude():
        async for position in drone.telemetry.position():
            if position.relative_altitude_m >= altitude_m * 0.9:
                return

    try:
        await asyncio.wait_for(_wait_altitude(), timeout=TIMEOUT_TAKEOFF_S)
        log.info("Reached takeoff altitude.")
    except asyncio.TimeoutError:
        raise RuntimeError(
            f"Timed out after {TIMEOUT_TAKEOFF_S}s waiting for takeoff altitude."
        )


# FIX: signature now takes home_alt_amsl. MAVSDK's goto_location expects
# ABSOLUTE (AMSL) altitude, not relative-to-home. Passing the relative
# value (10 m) against a home at ~500 m AMSL commands the drone to descend
# to ~10 m AMSL — i.e. into the ground. This was the critical bug.
async def goto_waypoint(drone: System, lat: float, lon: float, alt_m: float,
                        home_alt_amsl: float, yaw_deg: float = 0.0):
    """Maps to protocol.md command: GOTO { lat, lon, alt }.

    `alt_m` is relative-to-home (matches protocol.md). We convert to AMSL
    internally before calling MAVSDK.
    """
    target_amsl = home_alt_amsl + alt_m
    log.info(
        "GOTO lat=%.7f lon=%.7f alt_rel=%.1fm (alt_amsl=%.1fm) yaw=%.1f",
        lat, lon, alt_m, target_amsl, yaw_deg,
    )
    await drone.action.goto_location(lat, lon, target_amsl, yaw_deg)

    # FIX: bounded wait. Poll until close to target — a simple distance
    # check, not a precision approach controller.
    async def _wait_arrival():
        async for position in drone.telemetry.position():
            dist_m = _rough_distance_m(
                position.latitude_deg, position.longitude_deg, lat, lon
            )
            if dist_m < 2.0:
                return dist_m

    try:
        dist_m = await asyncio.wait_for(_wait_arrival(), timeout=TIMEOUT_GOTO_S)
        log.info("Reached waypoint (within %.1fm).", dist_m)
    except asyncio.TimeoutError:
        raise RuntimeError(
            f"Timed out after {TIMEOUT_GOTO_S}s flying to waypoint "
            f"({lat:.7f}, {lon:.7f}, rel {alt_m}m)."
        )


async def hold_position(seconds: float):
    """Maps to protocol.md command: HOLD. On the real vehicle this is just
    "stop sending new setpoints" — goto_location already leaves the drone
    loitering at the target, so this is a deliberate pause in the script,
    not an extra MAVSDK call.
    """
    log.info("HOLD for %.1fs", seconds)
    await asyncio.sleep(seconds)


async def return_and_land(drone: System):
    """Maps to protocol.md command: RTL."""
    log.info("RTL")
    try:
        await drone.action.return_to_launch()
    except ActionError as e:
        log.error("RTL failed: %s", e)
        raise

    # FIX: bounded wait for disarm (i.e. landed).
    async def _wait_disarmed():
        async for armed in drone.telemetry.armed():
            if not armed:
                return

    try:
        await asyncio.wait_for(_wait_disarmed(), timeout=TIMEOUT_LAND_S)
        log.info("Landed and disarmed.")
    except asyncio.TimeoutError:
        raise RuntimeError(
            f"Timed out after {TIMEOUT_LAND_S}s waiting for landing/disarm."
        )


def _rough_distance_m(lat1, lon1, lat2, lon2) -> float:
    """Flat-earth approximation, fine at the scale of a single waypoint
    check. Do not reuse this for field-scale coverage-area math in
    planning/ — use a proper geodesic calc there.
    """
    import math
    dlat = (lat2 - lat1) * 111_320
    dlon = (lon2 - lon1) * 111_320 * math.cos(math.radians(lat1))
    return math.hypot(dlat, dlon)


async def run_single_drone_mission(connection_string: str, altitude_m: float,
                                    wp_lat: float, wp_lon: float, wp_alt_m: float,
                                    yaw_deg: float):
    drone = await connect_drone(connection_string)
    home_alt_amsl = await get_home_altitude(drone)

    telemetry_task = asyncio.create_task(monitor_telemetry(drone))

    mission_completed = False
    try:
        await arm_and_takeoff(drone, altitude_m)
        await goto_waypoint(drone, wp_lat, wp_lon, wp_alt_m, home_alt_amsl, yaw_deg)
        await hold_position(5.0)
        await return_and_land(drone)
        mission_completed = True
    finally:
        # ADDED: safety net. Previously, any exception after takeoff left
        # the drone armed and airborne with nothing to bring it home.
        if not mission_completed:
            log.warning("Mission did not complete — issuing safety RTL.")
            try:
                await drone.action.return_to_launch()
            except Exception as e:
                log.error("Safety RTL failed: %s", e)

        # FIX: cancel *and* await the telemetry task, and swallow its
        # CancelledError so it doesn't propagate during shutdown.
        telemetry_task.cancel()
        try:
            await telemetry_task
        except asyncio.CancelledError:
            pass


def main():
    parser = argparse.ArgumentParser(description="Single-drone control test (roadmap step 4)")
    parser.add_argument(
        "--connection", default="udp://:14540",
        help="MAVSDK connection string. Default matches ArduPilot SITL "
             "(sim_vehicle.py --out=udp:127.0.0.1:14540). Use "
             "serial:///dev/ttyACM0:57600 for a real Pixhawk.",
    )
    parser.add_argument("--altitude", type=float, default=DEFAULT_TAKEOFF_ALT_M)
    parser.add_argument("--wp-lat", type=float, default=DEFAULT_WAYPOINT_LAT)
    parser.add_argument("--wp-lon", type=float, default=DEFAULT_WAYPOINT_LON)
    parser.add_argument("--wp-alt", type=float, default=DEFAULT_WAYPOINT_ALT_M)
    # ADDED: yaw control, needed once survey patterns land in planning/.
    parser.add_argument("--yaw", type=float, default=0.0,
                        help="Yaw at waypoint in degrees (0 = north).")
    args = parser.parse_args()

    _setup_file_logging()

    asyncio.run(run_single_drone_mission(
        args.connection, args.altitude,
        args.wp_lat, args.wp_lon, args.wp_alt, args.yaw,
    ))


if __name__ == "__main__":
    main()
