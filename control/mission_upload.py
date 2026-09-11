"""
control/mission_upload.py

Uploads a coverage plan (produced by planning/coverage.py) to a drone via
MAVSDK's mission API. Two modes:

  --upload-only   Connect, upload the mission, print status, exit. The
                  mission becomes visible in QGroundControl's Plan view,
                  but the drone does not arm or fly.

  (default)       Full flow: upload -> arm -> takeoff -> start mission ->
                  monitor progress -> RTL -> wait for landing.

Why use the mission API instead of goto_location()?

NIDAR rules (see docs/protocol.md) require the mission to be fully
pre-planned and uploaded before launch. No waypoint changes, no replanning
during flight. The mission API is how we satisfy that constraint — the
entire flight path lives on the Pixhawk before the drone leaves the ground.

Requires: mavsdk (see requirements.txt)

Usage:
    # Visualize in QGC only
    python -m control.mission_upload --upload-only --drone-index 0

    # Full flight test
    python -m control.mission_upload --drone-index 0
"""

import argparse
import asyncio
import logging
from datetime import datetime
from pathlib import Path

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.mission import MissionItem, MissionPlan

from planning.coverage import (
    make_field_from_launch,
    plan_mission,
)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("mission_upload")

TIMEOUT_CONNECT_S = 30.0
TIMEOUT_HEALTH_S = 60.0
TIMEOUT_UPLOAD_S = 60.0
TIMEOUT_TAKEOFF_S = 60.0
TIMEOUT_MISSION_S = 900.0
TIMEOUT_LAND_S = 120.0

DEFAULT_CRUISE_ALT_M = 10.0
DEFAULT_CRUISE_SPEED_MPS = 5.0
DEFAULT_ACCEPTANCE_RADIUS_M = 2.0

# Launch defaults — must match what planning/coverage.py uses, otherwise
# the drone will try to fly to a field that isn't where it thinks it is.
DEFAULT_LAUNCH_LAT = -35.3632620
DEFAULT_LAUNCH_LON = 149.1652373


def _setup_file_logging() -> None:
    log_dir = Path("logs")
    log_dir.mkdir(exist_ok=True)
    fname = log_dir / f"mission_upload_{datetime.now():%Y%m%d_%H%M%S}.log"
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
        raise RuntimeError(f"Timed out after {TIMEOUT_CONNECT_S}s waiting for connection.")

    log.info("Waiting for pre-arm health checks...")

    async def _wait_health():
        async for health in drone.telemetry.health():
            if (health.is_armable
                    and health.is_global_position_ok
                    and health.is_home_position_ok):
                return

    try:
        await asyncio.wait_for(_wait_health(), timeout=TIMEOUT_HEALTH_S)
        log.info("Pre-arm health OK.")
    except asyncio.TimeoutError:
        raise RuntimeError(f"Timed out after {TIMEOUT_HEALTH_S}s waiting for pre-arm health.")

    return drone


def build_mission_items(waypoints, cruise_speed_mps: float,
                        acceptance_radius_m: float):
    """Convert a list of (lat, lon, alt_m) into MAVSDK MissionItems.
    Uses float('nan') for fields where we want ArduPilot's defaults.

    NOTE: MAVSDK 3.x's VehicleAction enum does not expose RETURN_TO_LAUNCH,
    so the RTL step is issued separately from run_mission() after the
    last waypoint completes.
    """
    nan = float("nan")
    items = []
    for (lat, lon, alt_m) in waypoints:
        items.append(MissionItem(
            latitude_deg=lat,
            longitude_deg=lon,
            relative_altitude_m=alt_m,
            speed_m_s=cruise_speed_mps,
            is_fly_through=False,
            gimbal_pitch_deg=nan,
            gimbal_yaw_deg=nan,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=nan,
            camera_photo_interval_s=nan,
            acceptance_radius_m=acceptance_radius_m,
            yaw_deg=nan,
            camera_photo_distance_m=nan,
            vehicle_action=MissionItem.VehicleAction.NONE,
        ))
    return items


async def upload_mission(drone: System, items) -> None:
    log.info("Uploading mission: %d items", len(items))
    plan = MissionPlan(items)

    async def _do_upload():
        await drone.mission.upload_mission(plan)

    try:
        await asyncio.wait_for(_do_upload(), timeout=TIMEOUT_UPLOAD_S)
    except asyncio.TimeoutError:
        raise RuntimeError(f"Timed out after {TIMEOUT_UPLOAD_S}s during mission upload.")

    async def _read_count():
        async for progress in drone.mission.mission_progress():
            return progress.current + progress.total

    try:
        n = await asyncio.wait_for(_read_count(), timeout=10.0)
        log.info("Mission uploaded. Item count on vehicle: %d", n)
    except asyncio.TimeoutError:
        log.info("Mission uploaded (could not verify count).")


async def run_mission(drone: System, cruise_alt_m: float) -> None:
    """Full flight sequence: arm -> takeoff -> start mission -> wait ->
    RTL -> wait for landing."""
    log.info("ARM")
    try:
        await drone.action.arm()
    except ActionError as e:
        log.error("Arm failed: %s", e)
        raise

    log.info("TAKEOFF to %.1f m", cruise_alt_m)
    await drone.action.set_takeoff_altitude(cruise_alt_m)
    try:
        await drone.action.takeoff()
    except ActionError as e:
        log.error("Takeoff failed: %s", e)
        raise

    async def _wait_altitude():
        async for position in drone.telemetry.position():
            if position.relative_altitude_m >= cruise_alt_m * 0.9:
                return

    try:
        await asyncio.wait_for(_wait_altitude(), timeout=TIMEOUT_TAKEOFF_S)
        log.info("Reached takeoff altitude.")
    except asyncio.TimeoutError:
        raise RuntimeError("Timed out waiting for takeoff altitude.")

    log.info("Starting mission")
    try:
        await drone.mission.start_mission()
    except Exception as e:
        log.error("start_mission failed: %s", e)
        raise

    log.info("Monitoring mission progress...")

    async def _wait_complete():
        async for progress in drone.mission.mission_progress():
            log.info("Mission progress: %d / %d",
                     progress.current, progress.total)
            if progress.current == progress.total and progress.total > 0:
                return

    try:
        await asyncio.wait_for(_wait_complete(), timeout=TIMEOUT_MISSION_S)
        log.info("Mission complete.")
    except asyncio.TimeoutError:
        raise RuntimeError(f"Timed out after {TIMEOUT_MISSION_S}s waiting for mission completion.")

    # ArduPilot does not RTL automatically after a mission; it just hovers
    # in AUTO. Trigger RTL explicitly from here so the drone comes home.
    log.info("Triggering RTL")
    try:
        await drone.action.return_to_launch()
    except ActionError as e:
        log.error("RTL failed: %s", e)
        raise

    log.info("Waiting for landing/disarm...")

    async def _wait_disarmed():
        async for armed in drone.telemetry.armed():
            if not armed:
                return

    try:
        await asyncio.wait_for(_wait_disarmed(), timeout=TIMEOUT_LAND_S)
        log.info("Landed and disarmed.")
    except asyncio.TimeoutError:
        log.warning("Timed out waiting for disarm.")


async def run(args):
    field = make_field_from_launch(
        launch_lat=args.launch_lat,
        launch_lon=args.launch_lon,
        width_m=400.0,
        height_m=250.0,
        gap_m=30.0,
        side="south",
    )
    plans = plan_mission(
        field, n_drones=4,
        cruise_alt_m=args.altitude,
        camera_hfov_deg=args.hfov,
        overlap=args.overlap,
        safety_margin_m=0.0,
    )
    if args.drone_index < 0 or args.drone_index >= len(plans):
        raise SystemExit(f"--drone-index must be 0..{len(plans) - 1}")

    p = plans[args.drone_index]
    log.info("Using sector %d: %d waypoints, %.1f m total (search + transit)",
             p["index"], len(p["waypoints"]), p["total_distance_m"])

    items = build_mission_items(
        p["waypoints"], args.speed, DEFAULT_ACCEPTANCE_RADIUS_M
    )

    drone = await connect_drone(args.connection)

    try:
        await drone.mission.clear_mission()
        log.info("Cleared any existing mission on vehicle.")
    except Exception as e:
        log.warning("Could not clear mission: %s", e)

    await upload_mission(drone, items)

    if args.upload_only:
        log.info("--upload-only: not arming or flying. Mission is visible in QGC.")
        return

    await run_mission(drone, args.altitude)


def main():
    ap = argparse.ArgumentParser(description="Upload coverage mission to drone")
    ap.add_argument("--connection", default="udp://:14540")
    ap.add_argument("--drone-index", type=int, default=0,
                    help="Which sector from the 4-drone plan to upload (0-3)")
    ap.add_argument("--altitude", type=float, default=DEFAULT_CRUISE_ALT_M)
    ap.add_argument("--speed", type=float, default=DEFAULT_CRUISE_SPEED_MPS)
    ap.add_argument("--hfov", type=float, default=102.0)
    ap.add_argument("--overlap", type=float, default=0.30)
    ap.add_argument("--launch-lat", type=float, default=DEFAULT_LAUNCH_LAT,
                    help="Launch latitude — must match planning/coverage.py")
    ap.add_argument("--launch-lon", type=float, default=DEFAULT_LAUNCH_LON,
                    help="Launch longitude — must match planning/coverage.py")
    ap.add_argument("--upload-only", action="store_true",
                    help="Upload mission only; do not arm or fly.")
    args = ap.parse_args()

    _setup_file_logging()
    asyncio.run(run(args))


if __name__ == "__main__":
    main()