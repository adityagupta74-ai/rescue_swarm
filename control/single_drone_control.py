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

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("single_drone")

# Default target altitude for takeoff and the test waypoint (meters, relative to home)
DEFAULT_TAKEOFF_ALT_M = 10.0
# Default test waypoint offset — replace with a real field waypoint once
# docs/mission_spec.md has actual venue GPS bounds instead of TODO (venue)
DEFAULT_WAYPOINT_LAT = 47.397606
DEFAULT_WAYPOINT_LON = 8.543060
DEFAULT_WAYPOINT_ALT_M = 10.0


async def connect_drone(connection_string: str) -> System:
    """ARM/TAKEOFF/GOTO all require a connected System first. Blocks until
    the drone reports connected — this is the thing that hangs forever if
    SITL isn't running yet or the connection string is wrong, so it logs
    clearly rather than failing silently.
    """
    drone = System()
    log.info("Connecting to drone on %s ...", connection_string)
    await drone.connect(system_address=connection_string)

    async for state in drone.core.connection_state():
        if state.is_connected:
            log.info("Drone connected.")
            break

    log.info("Waiting for global position + home position lock (needed for GOTO/RTL)...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            log.info("Position lock OK.")
            break

    return drone


async def monitor_telemetry(drone: System):
    """Continuous telemetry read-back — runs concurrently with the mission
    steps below via asyncio.create_task(). Field names/comments map to
    docs/protocol.md's telemetry table.
    """
    async def watch_position():
        async for position in drone.telemetry.position():
            # -> protocol.md: gps { lat, lon, alt }
            log.info(
                "TELEMETRY gps: lat=%.7f lon=%.7f alt=%.2fm",
                position.latitude_deg, position.longitude_deg,
                position.relative_altitude_m,
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
    await drone.action.set_takeoff_altitude(altitude_m)
    await drone.action.takeoff()

    # Wait until we've actually reached (roughly) takeoff altitude before
    # sending the next command — sending GOTO too early is a common
    # source of flaky single-drone scripts.
    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= altitude_m * 0.9:
            log.info("Reached takeoff altitude.")
            break


async def goto_waypoint(drone: System, lat: float, lon: float, alt_m: float):
    """Maps to protocol.md command: GOTO { lat, lon, alt }."""
    log.info("GOTO lat=%.7f lon=%.7f alt=%.1fm", lat, lon, alt_m)
    await drone.action.goto_location(lat, lon, alt_m, yaw_deg=0)

    # Poll until close to the target — a simple distance check, not a
    # precision approach controller. Good enough for step 4; revisit if
    # the coverage algorithm (planning/) needs tighter waypoint tolerance.
    async for position in drone.telemetry.position():
        dist_m = _rough_distance_m(
            position.latitude_deg, position.longitude_deg, lat, lon
        )
        if dist_m < 2.0:
            log.info("Reached waypoint (within %.1fm).", dist_m)
            break


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

    async for armed in drone.telemetry.armed():
        if not armed:
            log.info("Landed and disarmed.")
            break


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
                                    wp_lat: float, wp_lon: float, wp_alt_m: float):
    drone = await connect_drone(connection_string)

    telemetry_task = asyncio.create_task(monitor_telemetry(drone))

    try:
        await arm_and_takeoff(drone, altitude_m)
        await goto_waypoint(drone, wp_lat, wp_lon, wp_alt_m)
        await hold_position(5.0)
        await return_and_land(drone)
    finally:
        telemetry_task.cancel()


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
    args = parser.parse_args()

    asyncio.run(run_single_drone_mission(
        args.connection, args.altitude, args.wp_lat, args.wp_lon, args.wp_alt
    ))


if __name__ == "__main__":
    main()
