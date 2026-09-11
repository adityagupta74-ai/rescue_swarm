"""
single_drone_control.py

SINGLE-DRONE control script — roadmap step 4 ("get single-drone control
rock solid" before moving to multi-drone coordination).

Scope of this file, on purpose:
    connect -> arm -> takeoff -> fly to one GPS waypoint -> hold -> RTL -> land
    + continuous telemetry read-back the whole time.

This is NOT the swarm coordinator. Multi-drone logic (segment assignment,
running 4 of these at once, battery-triggered auto-RTL across the fleet)
belongs in control/multi_drone_control.py later.

Commands used below map directly to docs/protocol.md's command table:
    ARM, TAKEOFF, GOTO, HOLD, RTL

Requires: mavsdk (see requirements.txt)

Run against ArduPilot SITL:
    1. Start SITL in a separate terminal:
         sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14540 --console --map
    2. Run this script:
         python control/single_drone_control.py

Run against a real Pixhawk (later):
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

DEFAULT_TAKEOFF_ALT_M = 10.0
# Default test waypoint — SITL's home is in Canberra (-35.3632620, 149.1652373).
# This waypoint is ~80 m northeast of home, reachable in a few seconds.
# Replace with real field waypoints once docs/mission_spec.md has venue GPS bounds.
DEFAULT_WAYPOINT_LAT = -35.362500
DEFAULT_WAYPOINT_LON = 149.165800
DEFAULT_WAYPOINT_ALT_M = 10.0

TIMEOUT_CONNECT_S = 30.0
TIMEOUT_HEALTH_S = 60.0
TIMEOUT_TAKEOFF_S = 60.0
TIMEOUT_GOTO_S = 180.0
TIMEOUT_LAND_S = 120.0


def _setup_file_logging() -> None:
    log_dir = Path("logs")
    log_dir.mkdir(exist_ok=True)
    fname = log_dir / f"single_drone_{datetime.now():%Y%m%d_%H%M%S}.log"
    handler = logging.FileHandler(fname)
    handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
    logging.getLogger().addHandler(handler)
    log.info("Logging to %s", fname)


async def connect_drone(connection_string: str) -> System:
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
        await asyncio.wait_for(_wait_health(), timeout=TIMEOUT_HEALTH_S)
    except asyncio.TimeoutError:
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


async def get_home_altitude(drone: System) -> float:
    """Read the drone's current AMSL altitude BEFORE takeoff. Since the
    drone is on the ground at this point, its AMSL altitude is effectively
    home altitude. This is more reliable than telemetry.home(), which
    ArduPilot does not publish until after the first arm.
    """
    async def _wait_position():
        async for position in drone.telemetry.position():
            return position.absolute_altitude_m

    try:
        home_alt = await asyncio.wait_for(_wait_position(), timeout=TIMEOUT_HEALTH_S)
        log.info("Home altitude (AMSL, from position): %.2f m", home_alt)
        return home_alt
    except asyncio.TimeoutError:
        raise RuntimeError("Timed out waiting for initial position fix.")


async def monitor_telemetry(drone: System):
    async def watch_position():
        async for position in drone.telemetry.position():
            log.info(
                "TELEMETRY gps: lat=%.7f lon=%.7f alt_rel=%.2fm alt_amsl=%.2fm",
                position.latitude_deg, position.longitude_deg,
                position.relative_altitude_m, position.absolute_altitude_m,
            )

    async def watch_battery():
        # NOTE: In MAVSDK 3.x, remaining_percent is already 0-100, not 0-1.
        async for battery in drone.telemetry.battery():
            log.info("TELEMETRY battery_pct: %.0f%%", battery.remaining_percent)

    async def watch_flight_mode():
        # NOTE: MAVSDK maps ArduPilot's GUIDED mode to FlightMode.OFFBOARD
        # in its abstract enum. Don't compare against FlightMode.GUIDED.
        async for mode in drone.telemetry.flight_mode():
            log.info("TELEMETRY mode: %s", mode)

    async def watch_armed():
        async for armed in drone.telemetry.armed():
            log.info("TELEMETRY armed: %s", armed)

    await asyncio.gather(
        watch_position(),
        watch_battery(),
        watch_flight_mode(),
        watch_armed(),
    )


async def arm_and_takeoff(drone: System, altitude_m: float):
    """ARM then TAKEOFF. MAVSDK's action.takeoff() internally switches the
    autopilot to GUIDED (or the vehicle-specific equivalent) before issuing
    MAV_CMD_NAV_TAKEOFF, so we don't need to set the mode explicitly.
    """
    log.info("ARM")
    try:
        await drone.action.arm()
    except ActionError as e:
        log.error("Arm failed: %s", e)
        raise

    log.info("TAKEOFF altitude=%.1fm", altitude_m)
    await drone.action.set_takeoff_altitude(altitude_m)
    try:
        await drone.action.takeoff()
    except ActionError as e:
        log.error("Takeoff failed: %s", e)
        raise

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


async def goto_waypoint(drone: System, lat: float, lon: float, alt_m: float,
                        home_alt_amsl: float, yaw_deg: float = 0.0):
    target_amsl = home_alt_amsl + alt_m
    log.info(
        "GOTO lat=%.7f lon=%.7f alt_rel=%.1fm (alt_amsl=%.1fm) yaw=%.1f",
        lat, lon, alt_m, target_amsl, yaw_deg,
    )
    await drone.action.goto_location(lat, lon, target_amsl, yaw_deg)

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
    log.info("HOLD for %.1fs", seconds)
    await asyncio.sleep(seconds)


async def return_and_land(drone: System):
    log.info("RTL")
    try:
        await drone.action.return_to_launch()
    except ActionError as e:
        log.error("RTL failed: %s", e)
        raise

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
        if not mission_completed:
            log.warning("Mission did not complete — issuing safety RTL.")
            try:
                await drone.action.return_to_launch()
            except Exception as e:
                log.error("Safety RTL failed: %s", e)

        telemetry_task.cancel()
        try:
            await telemetry_task
        except asyncio.CancelledError:
            pass


def main():
    parser = argparse.ArgumentParser(description="Single-drone control test")
    parser.add_argument(
        "--connection", default="udp://:14540",
        help="MAVSDK connection string. Default matches ArduPilot SITL.",
    )
    parser.add_argument("--altitude", type=float, default=DEFAULT_TAKEOFF_ALT_M)
    parser.add_argument("--wp-lat", type=float, default=DEFAULT_WAYPOINT_LAT)
    parser.add_argument("--wp-lon", type=float, default=DEFAULT_WAYPOINT_LON)
    parser.add_argument("--wp-alt", type=float, default=DEFAULT_WAYPOINT_ALT_M)
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