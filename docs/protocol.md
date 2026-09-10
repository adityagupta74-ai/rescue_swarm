# Drone State Machine & Message Protocol

Star topology: base station ↔ each drone independently. Drones never
communicate with each other. This doc defines the states each drone can
be in, and the exact messages that cross the wire in each direction.

## Drone states

| State | Meaning | Entered from | Exits to |
|---|---|---|---|
| `IDLE` | On the ground, disarmed, waiting for mission start | (initial) | `ARMING` |
| `ARMING` | Arm command sent, waiting for confirmation | `IDLE` | `TAKEOFF`, `IDLE` (arm failed) |
| `TAKEOFF` | Climbing to search altitude | `ARMING` | `SEARCHING` |
| `SEARCHING` | Flying assigned coverage pattern, running YOLO on camera feed | `TAKEOFF`, `INVESTIGATING` | `INVESTIGATING`, `RETURNING`, `FAILSAFE` |
| `INVESTIGATING` | Person flagged — holding position, confirming, reporting GPS | `SEARCHING` | `SEARCHING` (resume), `RETURNING` |
| `RETURNING` | Returning to launch (mission complete, battery low, or comms lost) | `SEARCHING`, `INVESTIGATING`, `FAILSAFE` | `LANDED` |
| `LANDED` | On the ground, mission ended | `RETURNING` | `IDLE` (reset for next run) |
| `FAILSAFE` | Something is wrong — hold or auto-RTL depending on trigger | any state | `RETURNING`, or holds until manual override takes over |

**FAILSAFE triggers:** comms link lost beyond timeout, battery below
threshold, geofence breach, GPS health degraded. Each trigger's specific
response (hold vs. immediate RTL) is a tuning decision to make during
simulation testing (step 6 in the roadmap) — record the chosen behavior
per trigger here once decided.

**Manual override:** at any state, an ELRS RC input from a human pilot
takes control directly at the Pixhawk level — this bypasses the state
machine entirely rather than transitioning through it, since RC exists
specifically to work even if the software state machine itself is
compromised. The base station's dashboard should still reflect "manual
override active" for that drone by watching for RC-override telemetry
flags, but it does not initiate or control the override.

## Message protocol (base ↔ drone, over WiFi/MAVLink; 433MHz radio
mirrors the telemetry-only messages as backup)

### Commands — base → drone

| Command | Payload | Purpose |
|---|---|---|
| `ARM` | — | Arm the drone |
| `DISARM` | — | Disarm (emergency stop, ground only) |
| `TAKEOFF` | `altitude` | Climb to search altitude |
| `GOTO` | `lat, lon, alt` | Navigate to a waypoint (used by the coverage algorithm) |
| `HOLD` | — | Hold current position (used when a detection is flagged) |
| `RTL` | — | Return to launch |
| `SET_MODE` | `mode` | Change flight mode |

### Telemetry — drone → base

| Field | Type | Purpose |
|---|---|---|
| `drone_id` | int | Which drone this telemetry is from |
| `state` | enum | Current state machine state (see table above) |
| `gps` | `lat, lon, alt` | Current position |
| `battery_pct` | float | Battery remaining |
| `heading` | float | Current heading |
| `mode` | string | Current flight mode |
| `rc_override_active` | bool | True if a human has taken manual control |
| `health` | struct | GPS lock quality, EKF status, link quality |
| `assigned_task` | string | Current sector/area assignment (rulebook requires the GCS to display each drone's assigned search area/task) |
| `kit_status` | enum | `LOADED`, `DELIVERED`, `EMPTY` — rulebook requires the GCS to show kit delivery status per drone |

### Detection events — drone → base

| Field | Type | Purpose |
|---|---|---|
| `drone_id` | int | Which drone found this |
| `gps` | `lat, lon` | Estimated location of the detected person |
| `confidence` | float | YOLO26n detection confidence |
| `timestamp` | datetime | When the detection occurred |
| `frame_ref` | string/bytes | Optional — rulebook only requires the geotag to be shown on the GCS, no confirming image/screenshot is mandated. Keep this field if you want it for your own debugging/scoring-dispute evidence, but it's not a rulebook requirement. |

**Note:** per the rulebook (see `mission_spec.md`), any manual survivor-
tagging input during the mission counts as a scored penalty (-50 per
instance). The `gps` field in this message must be the algorithm's own
autonomous output — never a value an operator clicks to confirm or
adjust mid-mission.
