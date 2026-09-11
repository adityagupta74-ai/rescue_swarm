"""
control/multi_drone_control.py

Swarm coordinator — connects to N drones (one per SITL instance / one per
real vehicle), uploads a sector-specific mission to each, then runs the
whole fleet in parallel: arm, takeoff, execute, RTL, land.

NIDAR compliance notes:
  - This is the SINGLE ground control process. All drones are coordinated
    through it — there is no independent per-drone control.
  - The mission is fully planned and uploaded before any drone arms.
  - After takeoff, this script only monitors and handles safety abort.
  - No waypoint change, no replan, no manual intervention during flight.

Requires: 4 SITL instances already running (see docs/protocol.md for the
launch commands), each exposing a MAVSDK connection at udp://:14540..14543.

Usage:
    python -m control.multi_drone_control
    python -m control.multi_drone_control --dry-run   # connect + upload only
"""

import argparse
import asyncio
import logging
from datetime import datetime
from pathlib import Path
from typing import List

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.mission import MissionItem, MissionPlan

from planning.coverage import make_field_from_launch, plan_mission

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("swarm")

# ---- tunables ----
DEFAULT_CONNECTIONS = [
    "udp://:14540",
    "udp://:14541",
    "udp://:14542",
    "udp://:14543",
]
DEFAULT_LAUNCH_LAT = -35.3632620
DEFAULT_LAUNCH_LON = 149.1652373
DEFAULT_ALT_M = 10.0
DEFAULT_SPEED_MPS = 5.0
ACCEPTANCE_RADIUS_M = 2.0
TAKEOFF_STAGGER_S = 2.0   # seconds between each drone's takeoff command

# Each System() spawns its own mavsdk_server subprocess. If they all try to
# bind to gRPC port 50051, they collide and mission uploads start failing
# with BUSY errors. Assign a distinct port per drone: 50051 + index.
BASE_GRPC_PORT = 50051

TIMEOUT_CONNECT_S = 30.0
TIMEOUT_HEALTH_S = 60.0
TIMEOUT_UPLOAD_S = 60.0
TIMEOUT_TAKEOFF_S = 60.0
TIMEOUT_MISSION_S = 900.0
TIMEOUT_LAND_S = 180.0


def _setup_file_logging() -> None:
    log_dir = Path("logs")
    log_dir.mkdir(exist_ok=True)
    fname = log_dir / f"swarm_{datetime.now():%Y%m%d_%H%M%S}.log"
    handler = logging.FileHandler(fname)
    handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
    logging.getLogger().addHandler(handler)
    log.info("Logging to %s", fname)


# ---------------- per-drone runner ----------------

class DroneRunner:
    """Owns one MAVSDK System and its connection to a single vehicle.
    All operations assume a connection has been established by connect()."""

    def __init__(self, index: int, connection_string: str, grpc_port: int):
        self.index = index
        self.connection_string = connection_string
        # Each System spawns its own mavsdk_server subprocess. They must
        # listen on distinct gRPC ports or they collide (BUSY errors on
        # concurrent mission uploads).
        self.drone = System(port=grpc_port)
        self._connected = False

    def _tag(self, msg: str) -> str:
        return f"[D{self.index}] {msg}"

    async def connect(self) -> None:
        log.info(self._tag(f"Connecting to {self.connection_string} ..."))
        await self.drone.connect(system_address=self.connection_string)

        async def _wait_connected():
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    return

        try:
            await asyncio.wait_for(_wait_connected(), timeout=TIMEOUT_CONNECT_S)
        except asyncio.TimeoutError:
            raise RuntimeError(self._tag(
                f"Timed out after {TIMEOUT_CONNECT_S}s waiting for connection"))

        self._connected = True
        log.info(self._tag("Connected."))

        async def _wait_health():
            async for h in self.drone.telemetry.health():
                if (h.is_armable and h.is_global_position_ok
                        and h.is_home_position_ok):
                    return

        try:
            await asyncio.wait_for(_wait_health(), timeout=TIMEOUT_HEALTH_S)
        except asyncio.TimeoutError:
            raise RuntimeError(self._tag(
                f"Timed out after {TIMEOUT_HEALTH_S}s waiting for health"))
        log.info(self._tag("Pre-arm health OK."))

    def build_mission(self, waypoints):
        nan = float("nan")
        return [MissionItem(
            latitude_deg=lat,
            longitude_deg=lon,
            relative_altitude_m=alt,
            speed_m_s=DEFAULT_SPEED_MPS,
            is_fly_through=False,
            gimbal_pitch_deg=nan,
            gimbal_yaw_deg=nan,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=nan,
            camera_photo_interval_s=nan,
            acceptance_radius_m=ACCEPTANCE_RADIUS_M,
            yaw_deg=nan,
            camera_photo_distance_m=nan,
            vehicle_action=MissionItem.VehicleAction.NONE,
        ) for (lat, lon, alt) in waypoints]

    async def upload(self, items) -> None:
        log.info(self._tag(f"Uploading mission ({len(items)} items)"))
        try:
            await self.drone.mission.clear_mission()
        except Exception:
            pass

        async def _do():
            await self.drone.mission.upload_mission(MissionPlan(items))

        try:
            await asyncio.wait_for(_do(), timeout=TIMEOUT_UPLOAD_S)
        except asyncio.TimeoutError:
            raise RuntimeError(self._tag("Mission upload timed out"))
        log.info(self._tag("Mission uploaded."))

    async def arm_takeoff(self, alt_m: float) -> None:
        log.info(self._tag("ARM"))
        try:
            await self.drone.action.arm()
        except ActionError as e:
            raise RuntimeError(self._tag(f"Arm failed: {e}"))

        log.info(self._tag(f"TAKEOFF to {alt_m:.1f} m"))
        await self.drone.action.set_takeoff_altitude(alt_m)
        try:
            await self.drone.action.takeoff()
        except ActionError as e:
            raise RuntimeError(self._tag(f"Takeoff failed: {e}"))

        async def _wait_alt():
            async for p in self.drone.telemetry.position():
                if p.relative_altitude_m >= alt_m * 0.9:
                    return

        try:
            await asyncio.wait_for(_wait_alt(), timeout=TIMEOUT_TAKEOFF_S)
        except asyncio.TimeoutError:
            raise RuntimeError(self._tag("Takeoff altitude not reached"))
        log.info(self._tag("At takeoff altitude."))

    async def start_and_monitor(self) -> None:
        log.info(self._tag("START mission"))
        try:
            await self.drone.mission.start_mission()
        except Exception as e:
            raise RuntimeError(self._tag(f"start_mission failed: {e}"))

        async def _wait_complete():
            last = -1
            async for progress in self.drone.mission.mission_progress():
                if progress.current != last:
                    log.info(self._tag(
                        f"progress {progress.current}/{progress.total}"))
                    last = progress.current
                if (progress.current == progress.total
                        and progress.total > 0):
                    return

        try:
            await asyncio.wait_for(_wait_complete(), timeout=TIMEOUT_MISSION_S)
        except asyncio.TimeoutError:
            raise RuntimeError(self._tag("Mission timeout"))
        log.info(self._tag("Mission complete."))

    async def rtl_and_wait(self) -> None:
        log.info(self._tag("RTL"))
        try:
            await self.drone.action.return_to_launch()
        except ActionError as e:
            log.error(self._tag(f"RTL failed: {e}"))
            return

        async def _wait_disarmed():
            async for armed in self.drone.telemetry.armed():
                if not armed:
                    return

        try:
            await asyncio.wait_for(_wait_disarmed(), timeout=TIMEOUT_LAND_S)
            log.info(self._tag("Landed and disarmed."))
        except asyncio.TimeoutError:
            log.warning(self._tag("Timed out waiting for landing."))

    async def emergency_rtl(self) -> None:
        """Best-effort RTL — used from the top-level exception handler."""
        try:
            await self.drone.action.return_to_launch()
            log.info(self._tag("Emergency RTL issued."))
        except Exception as e:
            log.error(self._tag(f"Emergency RTL failed: {e}"))


# ---------------- orchestration ----------------

async def run_swarm(connections: List[str], launch_lat: float, launch_lon: float,
                    alt_m: float, dry_run: bool) -> None:
    n = len(connections)
    if n < 2:
        raise SystemExit("Swarm needs at least 2 drones (NIDAR rule).")

    # 1. Plan the field once.
    field = make_field_from_launch(
        launch_lat=launch_lat, launch_lon=launch_lon,
        width_m=400.0, height_m=250.0, gap_m=30.0, side="south",
    )
    plans = plan_mission(
        field, n_drones=n, cruise_alt_m=alt_m,
        camera_hfov_deg=102.0, overlap=0.30, safety_margin_m=0.0,
    )
    log.info("Planned %d sectors. Longest drone: %.1f m",
             n, max(p["total_distance_m"] for p in plans))

    # 2. Connect all drones in parallel. Each gets a distinct gRPC port.
    runners = [DroneRunner(i, connections[i], grpc_port=BASE_GRPC_PORT + i)
               for i in range(n)]
    await asyncio.gather(*(r.connect() for r in runners))

    # 3. Upload all missions in parallel.
    await asyncio.gather(*(
        runners[i].upload(runners[i].build_mission(plans[i]["waypoints"]))
        for i in range(n)
    ))

    if dry_run:
        log.info("--dry-run: connected + uploaded. Not arming or flying.")
        return

    # 4. Arm + takeoff, staggered slightly to avoid overwhelming the GCS link.
    try:
        takeoff_tasks = []
        for i, r in enumerate(runners):
            async def delayed(runner=r, delay=i * TAKEOFF_STAGGER_S):
                if delay > 0:
                    await asyncio.sleep(delay)
                await runner.arm_takeoff(alt_m)
            takeoff_tasks.append(asyncio.create_task(delayed()))
        await asyncio.gather(*takeoff_tasks)

        # 5. All at altitude. Start missions in parallel.
        await asyncio.gather(*(r.start_and_monitor() for r in runners))

        # 6. Each drone issues its own RTL, all in parallel.
        await asyncio.gather(*(r.rtl_and_wait() for r in runners))

        log.info("Swarm mission complete.")
    except Exception as e:
        log.error("Swarm mission failed: %s", e)
        log.warning("Triggering emergency RTL on all drones.")
        await asyncio.gather(*(r.emergency_rtl() for r in runners),
                             return_exceptions=True)
        raise


def main():
    ap = argparse.ArgumentParser(description="RescueSwarm multi-drone coordinator")
    ap.add_argument("--connections", nargs="+", default=DEFAULT_CONNECTIONS,
                    help="MAVSDK connection strings, one per drone")
    ap.add_argument("--launch-lat", type=float, default=DEFAULT_LAUNCH_LAT)
    ap.add_argument("--launch-lon", type=float, default=DEFAULT_LAUNCH_LON)
    ap.add_argument("--altitude", type=float, default=DEFAULT_ALT_M)
    ap.add_argument("--dry-run", action="store_true",
                    help="Connect + upload only; do not fly")
    args = ap.parse_args()

    _setup_file_logging()
    asyncio.run(run_swarm(args.connections, args.launch_lat, args.launch_lon,
                          args.altitude, args.dry_run))


if __name__ == "__main__":
    main()