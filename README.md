# RescueSwarm — NIDAR 2026-27

Centralized 4-drone search-and-rescue swarm. Pixhawk 2.4.8 + Raspberry Pi 5 per
drone, Python coordination algorithm running on the base station, star
topology (base ↔ each drone; drones never talk to each other).

## Comms stack
- **Video + primary telemetry/commands:** WiFi network (router + antenna, base station side)
- **Backup telemetry:** 433MHz SiK-style radio, one pair per drone, distinct NetID per pair
- **Manual RC override:** ELRS, 1 transmitter (multi-model bind) + 1 receiver per drone
- **Onboard detection:** YOLO26n (person class), fine-tuned from COCO weights

## Build order (do not skip ahead — simulate before real hardware)
1. `docs/mission_spec.md` — rulebook requirements, written down before any code
2. `docs/protocol.md` — drone state machine + command/telemetry message format
3. `sim/` — SITL setup, single simulated drone
4. `control/` — single-drone control (connect, arm, takeoff, waypoint, land, telemetry read)
5. `planning/` — coverage/search pattern, waypoint generation for the field
6. `control/` — scale to 4 simulated drones, sector assignment, battery RTL, collision avoidance
7. `cv/` — YOLO26n integration into the mission decision loop
8. `dashboard/` — live feed + telemetry display, failsafes, then real hardware

## Folder guide
| Folder | Contents |
|---|---|
| `docs/` | mission spec, protocol/state-machine definitions |
| `sim/` | SITL launch scripts, simulation configs |
| `control/` | drone control logic (MAVSDK/pymavlink), single- and multi-drone |
| `planning/` | search pattern / coverage / waypoint generation |
| `cv/` | YOLO26n inference + detection-to-mission-event handling |
| `comms/` | radio/telemetry link handling, message schemas |
| `dashboard/` | live video + telemetry UI |

## Setup
```bash
python -m venv venv
source venv/bin/activate   # or venv\Scripts\activate on Windows
pip install -r requirements.txt
```
