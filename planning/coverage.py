"""
planning/coverage.py

10-hectare field decomposition + boustrophedon search-pattern generation
for the NIDAR RescueSwarm mission.

Given:
  - a launch position (outside the field)
  - a 10-ha rectangular field, 30m+ away from the launch
  - the number of drones N
  - the search altitude and camera horizontal FOV

Produces:
  - N equal-area sectors (horizontal strips; lanes run along the long axis)
  - one boustrophedon (lawnmower) waypoint list per sector
  - waypoints converted to (lat, lon, alt_m) ready for mission upload
  - a PNG plot showing the field boundary, the launch area, and all N paths

Design notes:
  - All planning happens offline, before mission start (NIDAR rule).
  - Each drone flies from launch to its first waypoint, runs the serpentine,
    then Python issues RTL to bring it back to launch.
  - Launch area is intentionally outside the field so takeoffs and landings
    don't interfere with the search area.

Usage:
    python planning/coverage.py                       # print demo
    python planning/coverage.py --plot                # save PNG
    python planning/coverage.py --launch-side north --launch-gap 40 --plot
"""

import argparse
import math
from dataclasses import dataclass
from typing import List, Tuple

METERS_PER_DEG_LAT = 111_320.0
DEFAULT_LAUNCH_GAP_M = 30.0   # distance from launch to nearest field edge
DEFAULT_FIELD_W_M = 400.0
DEFAULT_FIELD_H_M = 250.0
DEFAULT_LAUNCH_SIDE = "south"


# ---------- rotation helper ----------

def _rotate_xy(x: float, y: float, theta_rad: float) -> Tuple[float, float]:
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return x * c - y * s, x * s + y * c


# ---------- coordinate frame ----------

@dataclass
class LocalFrame:
    """Local ENU frame anchored at a lat/lon origin. Flat-earth approx —
    valid to ~1 cm accuracy at a few hundred meters scale."""
    origin_lat: float
    origin_lon: float

    def to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        y = (lat - self.origin_lat) * METERS_PER_DEG_LAT
        x = (lon - self.origin_lon) * METERS_PER_DEG_LAT * math.cos(
            math.radians(self.origin_lat))
        return x, y

    def to_lat_lon(self, x: float, y: float) -> Tuple[float, float]:
        lat = self.origin_lat + y / METERS_PER_DEG_LAT
        lon = self.origin_lon + x / (
            METERS_PER_DEG_LAT * math.cos(math.radians(self.origin_lat)))
        return lat, lon


# ---------- field ----------

@dataclass
class Field:
    """Rectangular field defined by its four corners, in order (CW or CCW).
    Corners are (lat, lon) tuples.

    A `launch_latlon` attribute is carried alongside the field so plots and
    mission generation can reference it. Launch is NOT part of the field."""
    corners_latlon: List[Tuple[float, float]]
    launch_latlon: Tuple[float, float]

    def __post_init__(self):
        n = len(self.corners_latlon)
        self.center_lat = sum(c[0] for c in self.corners_latlon) / n
        self.center_lon = sum(c[1] for c in self.corners_latlon) / n
        self.frame = LocalFrame(self.center_lat, self.center_lon)
        self.local_corners = [
            self.frame.to_local(lat, lon) for lat, lon in self.corners_latlon
        ]
        self.long_yaw_rad = self._long_edge_yaw()

    def _long_edge_yaw(self) -> float:
        pts = self.local_corners
        n = len(pts)
        best_len, best_angle = 0.0, 0.0
        for i in range(n):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % n]
            L = math.hypot(x2 - x1, y2 - y1)
            if L > best_len:
                best_len = L
                best_angle = math.atan2(y2 - y1, x2 - x1)
        return best_angle

    def area_ha(self) -> float:
        pts = self.local_corners
        n = len(pts)
        s = 0.0
        for i in range(n):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % n]
            s += x1 * y2 - x2 * y1
        return abs(s) / 2.0 / 10_000.0

    def bbox_in_lane_frame(self) -> Tuple[float, float, float, float]:
        yaw = -self.long_yaw_rad
        rotated = [_rotate_xy(x, y, yaw) for (x, y) in self.local_corners]
        xs = [p[0] for p in rotated]
        ys = [p[1] for p in rotated]
        return min(xs), min(ys), max(xs), max(ys)

    def lane_frame_to_lat_lon(self, x: float, y: float) -> Tuple[float, float]:
        x_loc, y_loc = _rotate_xy(x, y, self.long_yaw_rad)
        return self.frame.to_lat_lon(x_loc, y_loc)

    def launch_local(self) -> Tuple[float, float]:
        """Launch position in the field's local frame (meters)."""
        return self.frame.to_local(*self.launch_latlon)


# ---------- swath and boustrophedon ----------

def lane_spacing_from_camera(altitude_m: float, hfov_deg: float,
                              overlap_fraction: float) -> float:
    hfov_rad = math.radians(hfov_deg)
    swath = 2.0 * altitude_m * math.tan(hfov_rad / 2.0)
    return swath * (1.0 - overlap_fraction)


def boustrophedon_waypoints(x_min: float, x_max: float,
                             y_min: float, y_max: float,
                             lane_spacing_m: float,
                             margin_m: float = 0.0) -> Tuple[List[Tuple[float, float]], int]:
    xa = x_min + margin_m
    xb = x_max - margin_m
    ya = y_min + margin_m
    yb = y_max - margin_m

    lanes = []
    y = ya + lane_spacing_m / 2.0
    i = 0
    while y <= yb - lane_spacing_m / 2.0 + 1e-6:
        if i % 2 == 0:
            lanes.append([(xa, y), (xb, y)])
        else:
            lanes.append([(xb, y), (xa, y)])
        y += lane_spacing_m
        i += 1

    wps = []
    for lane in lanes:
        wps.extend(lane)
    return wps, len(lanes)


# ---------- planning entry ----------

def divide_into_sectors(field: Field, n: int) -> List[dict]:
    x_min, y_min, x_max, y_max = field.bbox_in_lane_frame()
    strip_h = (y_max - y_min) / n
    sectors = []
    for i in range(n):
        sectors.append({
            "index": i,
            "x_min": x_min, "x_max": x_max,
            "y_min": y_min + i * strip_h,
            "y_max": y_min + (i + 1) * strip_h,
        })
    return sectors


def plan_mission(field: Field, n_drones: int, cruise_alt_m: float,
                 camera_hfov_deg: float, overlap: float,
                 safety_margin_m: float = 0.0) -> List[dict]:
    lane_spacing = lane_spacing_from_camera(cruise_alt_m, camera_hfov_deg, overlap)
    sectors = divide_into_sectors(field, n_drones)
    launch_local = field.launch_local()

    plans = []
    for s in sectors:
        local_wps, n_lanes = boustrophedon_waypoints(
            s["x_min"], s["x_max"], s["y_min"], s["y_max"],
            lane_spacing, margin_m=safety_margin_m,
        )
        latlon_wps = []
        for (x, y) in local_wps:
            lat, lon = field.lane_frame_to_lat_lon(x, y)
            latlon_wps.append((lat, lon, cruise_alt_m))

        search_d = 0.0
        for i in range(len(local_wps) - 1):
            x1, y1 = local_wps[i]
            x2, y2 = local_wps[i + 1]
            search_d += math.hypot(x2 - x1, y2 - y1)

        transit_in = math.hypot(
            local_wps[0][0] - launch_local[0],
            local_wps[0][1] - launch_local[1])
        transit_out = math.hypot(
            local_wps[-1][0] - launch_local[0],
            local_wps[-1][1] - launch_local[1])

        plans.append({
            "index": s["index"],
            "sector_bounds": (s["x_min"], s["y_min"], s["x_max"], s["y_max"]),
            "waypoints": latlon_wps,
            "n_lanes": n_lanes,
            "search_distance_m": search_d,
            "transit_in_m": transit_in,
            "transit_out_m": transit_out,
            "total_distance_m": search_d + transit_in + transit_out,
            "lane_spacing_m": lane_spacing,
        })
    return plans


# ---------- field construction from launch position ----------

def make_field_from_launch(launch_lat: float, launch_lon: float,
                            width_m: float = DEFAULT_FIELD_W_M,
                            height_m: float = DEFAULT_FIELD_H_M,
                            gap_m: float = DEFAULT_LAUNCH_GAP_M,
                            side: str = DEFAULT_LAUNCH_SIDE) -> Field:
    """Construct a rectangular field positioned so that the launch point
    sits `gap_m` meters outside the field, centered on the given side.

    `side` is one of: "south", "north", "east", "west".
    """
    # Field center in the local frame anchored at launch.
    if side == "south":
        # Field is NORTH of launch. Bottom edge is at +gap_m.
        center_x, center_y = 0.0, gap_m + height_m / 2.0
    elif side == "north":
        # Field is SOUTH of launch. Top edge is at -gap_m.
        center_x, center_y = 0.0, -(gap_m + height_m / 2.0)
    elif side == "east":
        # Field is WEST of launch. Right edge is at -gap_m.
        center_x, center_y = -(gap_m + width_m / 2.0), 0.0
    elif side == "west":
        center_x, center_y = (gap_m + width_m / 2.0), 0.0
    else:
        raise ValueError(f"Unknown launch side: {side}")

    frame = LocalFrame(launch_lat, launch_lon)

    # For E/W launch, rotate the field so its long axis runs north-south
    # (i.e. swap which dimension the field's "width" applies to).
    if side in ("south", "north"):
        half_w, half_h = width_m / 2.0, height_m / 2.0
        corner_offsets = [
            (-half_w, -half_h), (half_w, -half_h),
            ( half_w,  half_h), (-half_w,  half_h),
        ]
    else:  # east / west
        half_w, half_h = width_m / 2.0, height_m / 2.0
        corner_offsets = [
            (-half_h, -half_w), (half_h, -half_w),
            ( half_h,  half_w), (-half_h,  half_w),
        ]

    # BUGFIX: was `cx`, `cy` — should be `center_x`, `center_y`
    local_corners = [(center_x + ox, center_y + oy) for (ox, oy) in corner_offsets]
    latlon_corners = [frame.to_lat_lon(x, y) for (x, y) in local_corners]

    return Field(latlon_corners, (launch_lat, launch_lon))


# ---------- reporting ----------

def print_summary(field: Field, plans: List[dict],
                  cruise_speed_mps: float = 5.0):
    launch_lat, launch_lon = field.launch_latlon
    cx_min, cy_min, cx_max, cy_max = field.bbox_in_lane_frame()

    print()
    print("=" * 62)
    print("FIELD")
    print("=" * 62)
    print(f"  area:          {field.area_ha():.2f} ha")
    print(f"  center:        {field.center_lat:.7f}, {field.center_lon:.7f}")
    print(f"  size (lane):   {cx_max - cx_min:.1f} m x {cy_max - cy_min:.1f} m")
    print()
    print("=" * 62)
    print("LAUNCH")
    print("=" * 62)
    print(f"  position:      {launch_lat:.7f}, {launch_lon:.7f}")
    lx, ly = field.launch_local()
    print(f"  field-rel:     ({lx:.1f}, {ly:.1f}) m from field center")
    print()
    print("=" * 62)
    print(f"DRONES ({len(plans)})")
    print("=" * 62)
    total_d = 0.0
    max_d = 0.0
    for p in plans:
        x_min, y_min, x_max, y_max = p["sector_bounds"]
        sx, sy = x_max - x_min, y_max - y_min
        t_total = p["total_distance_m"] / cruise_speed_mps
        total_d += p["total_distance_m"]
        max_d = max(max_d, p["total_distance_m"])
        print(f"  Drone {p['index']}: sector {sx:.0f} x {sy:.0f} m, "
              f"{p['n_lanes']} lanes")
        print(f"    transit in:   {p['transit_in_m']:.1f} m")
        print(f"    search:       {p['search_distance_m']:.1f} m")
        print(f"    transit out:  {p['transit_out_m']:.1f} m")
        print(f"    TOTAL:        {p['total_distance_m']:.1f} m  "
              f"({t_total:.1f} s @ {cruise_speed_mps:.1f} m/s)")
    print()
    print(f"  fleet total:   {total_d:.1f} m")
    print(f"  longest drone: {max_d:.1f} m  ({max_d / cruise_speed_mps:.1f} s)")
    print(f"  wall time:     {max_d / cruise_speed_mps:.1f} s "
          f"(parallel, no takeoff stagger)")
    print()


def plot_plan(field: Field, plans: List[dict], out_path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(13, 8))

    bx = [p[0] for p in field.local_corners] + [field.local_corners[0][0]]
    by = [p[1] for p in field.local_corners] + [field.local_corners[0][1]]
    ax.plot(bx, by, "k-", linewidth=2.2, label="10-ha field boundary")

    lx, ly = field.launch_local()
    half = 3.66 / 2.0
    ax.add_patch(Rectangle((lx - half, ly - half), 2 * half, 2 * half,
                            facecolor="green", edgecolor="black",
                            linewidth=1.5, alpha=0.65,
                            label="Launch area (12 ft x 12 ft)"))
    ax.text(lx, ly - 12, "LAUNCH", ha="center", va="top",
            fontsize=9, fontweight="bold", color="darkgreen")

    colors = ["#e6194B", "#3cb44b", "#4363d8", "#f58231",
              "#911eb4", "#42d4f4", "#f032e6", "#bfef45"]

    for i, p in enumerate(plans):
        wps_local = [field.frame.to_local(lat, lon)
                     for (lat, lon, _alt) in p["waypoints"]]
        xs = [w[0] for w in wps_local]
        ys = [w[1] for w in wps_local]
        color = colors[i % len(colors)]
        ax.plot(xs, ys, "-", color=color, linewidth=1.4,
                label=f"Drone {i} ({(p['total_distance_m'] / 5.0):.0f} s)")
        ax.plot(xs, ys, "o", color=color, markersize=3.5)
        ax.plot(xs[0], ys[0], "s", color=color, markersize=11,
                markeredgecolor="black", markeredgewidth=1.4)

    ax.set_aspect("equal")
    ax.set_xlabel("East (m) from field center")
    ax.set_ylabel("North (m) from field center")
    ax.set_title("NIDAR RescueSwarm — 10-ha field, 4 sectors, launch outside field")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, linestyle="--", alpha=0.4)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"Saved plot: {out_path}")


# ---------- main ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drones", type=int, default=4)
    ap.add_argument("--altitude", type=float, default=10.0,
                    help="Cruise/search altitude in meters")
    ap.add_argument("--hfov", type=float, default=102.0,
                    help="Camera HFOV in degrees")
    ap.add_argument("--overlap", type=float, default=0.30,
                    help="Swath overlap fraction between adjacent lanes")
    ap.add_argument("--launch-lat", type=float, default=-35.3632620,
                    help="Launch latitude (default = SITL home)")
    ap.add_argument("--launch-lon", type=float, default=149.1652373,
                    help="Launch longitude (default = SITL home)")
    ap.add_argument("--launch-side", default=DEFAULT_LAUNCH_SIDE,
                    choices=["south", "north", "east", "west"],
                    help="Which side of the field the launch area is on")
    ap.add_argument("--launch-gap", type=float, default=DEFAULT_LAUNCH_GAP_M,
                    help="Distance from launch to nearest field edge (m)")
    ap.add_argument("--field-w", type=float, default=DEFAULT_FIELD_W_M,
                    help="Field long dimension (m)")
    ap.add_argument("--field-h", type=float, default=DEFAULT_FIELD_H_M,
                    help="Field short dimension (m)")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--out-plot", default="docs/field_layout.png")
    args = ap.parse_args()

    field = make_field_from_launch(
        launch_lat=args.launch_lat,
        launch_lon=args.launch_lon,
        width_m=args.field_w,
        height_m=args.field_h,
        gap_m=args.launch_gap,
        side=args.launch_side,
    )

    plans = plan_mission(
        field, n_drones=args.drones,
        cruise_alt_m=args.altitude,
        camera_hfov_deg=args.hfov,
        overlap=args.overlap,
        safety_margin_m=0.0,
    )

    print_summary(field, plans)

    if args.plot:
        plot_plan(field, plans, args.out_plot)


if __name__ == "__main__":
    main()