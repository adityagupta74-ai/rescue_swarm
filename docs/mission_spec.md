# Mission Spec — RescueSwarm (NIDAR 2026-27, PS1, Rulebook v2.1)

Source: NIDAR-26-27-Rulebook-Ver_2_1.pdf, Annexure 1 (RescueSwarm brief) +
main body sections 4, 8, 9. Anything still marked **TODO (venue)** isn't
in the rulebook — it's set on the day, at the venue.

## Mission
- Scenario: flash flood, semi-urban settlement, telecom/mobile networks
  down — the "no external network" rule (below) is the scenario's actual
  premise, not an arbitrary restriction.
- Goal: find survivors and deliver a survivor kit to each, as fast as
  possible, through a single ground control system, single operator,
  minimal human intervention.
- Centralized control architecture (Python at base, star topology,
  drones never talk to each other) — this project's own design choice,
  compatible with the rules as written.

## Search area
- Up to 10 hectares, up to **10 survivors** present.
- Exact field shape/dimensions: **TODO (venue)** — not fixed in the
  rulebook, set by the organisers on the day. Design the coverage
  algorithm to take field bounds as a runtime input, not a hardcoded
  assumption.
- Base station position relative to the field: **TODO (venue)** —
  likewise set on the day.
- Launch/landing: all drones take off from and land within a fixed
  **12 ft x 12 ft** pad.

## Drone constraints
- Minimum 2 drones (this project uses 4) operating as one coordinated
  system — not independently controlled units.
- **Combined all-up weight of all deployed drones <= 25 kg** — batteries,
  payloads, sensors, comms equipment, everything, summed across all 4
  drones, not per drone. Check this against the actual build weight once
  hardware is finalized — this is a real risk line item, not a formality.
- No ready-to-fly/market-complete airframes — must be self-built
  (S500 frames + custom integration already satisfies this).

## Survivor kit / payload
- Each kit: **200g, 20cm x 10cm x 5cm**, rectangular box.
- Delivery must be autonomous (no manual drop command — see "manual
  intervention" below).
- Delivery accuracy scoring zones: within 1m = 20 pts/drop, within 2m =
  14 pts/drop, within 3m = 8 pts/drop, max 10 successful drops (200 pts
  total).

## Victim detection reporting
- Detection must be autonomous: geotag each survivor, display the
  location on the Ground Control Station.
- Scoring: 25 pts per correctly detected + correctly geotagged survivor,
  max 10 survivors (250 pts total).
- No specific format beyond "geotagged and shown on the GCS" — no
  screenshot/confidence-threshold requirement stated. Confirmation
  (human sign-off vs. fully autonomous) isn't mandated either way, but
  any manual survivor-tagging input during the mission counts as manual
  intervention (penalty) — so tagging must be the algorithm's own
  output, not something an operator clicks to confirm.

## Timing / scoring
- Final Mission time limit: **30 minutes** (points earned after expiry
  don't count).
- Setup time: 5 minutes before the mission timer may be started at the
  jury's discretion.
- Total scoring: 1000 pts — Design Review 200, Business Strategy 200,
  Pre-Flight Inspection pass/fail (gates you into the Final Mission),
  Final Mission 600.
- Final Mission breakdown: survivor detection+geotag 250, kit delivery
  accuracy 200, multi-drone collaborative execution (yes/no) 50, single
  GCS/unified interface (yes/no) 50, fast-completion bonus (finish within
  half the time limit) 50.
- Penalties: landing outside the pad -10/drone; geofence breach
  -20/instance (+additional -20 if the same drone repeats it); **manual
  intervention -50/instance**; crash -50/instance. Penalties capped at
  150 total, except safety-critical violations (uncapped, can mean
  mission termination or disqualification).

## What counts as "manual intervention" (-50 each, capped at 150 total)
Any of: manual waypoint modification, flight-path correction, payload-
release command, survivor tagging, or mission replanning **during
mission execution**. Safety abort and emergency recall are explicitly
**not** manual intervention — they're permitted without penalty.
**This directly shapes how the ELRS RC override gets used:** treat it as
an emergency-only recall/abort mechanism, not a casual "nudge the drone"
tool — using it to correct a flight path or retrigger a drop mid-mission
is a scored penalty, not a free safety net.

## Communication requirements
- **GSM, LTE, 5G, public Wi-Fi, internet, and cloud services are
  prohibited during mission execution.** All drone <-> onboard <-> GCS
  communication must run over **locally deployed** communication
  systems — this is exactly what the router+antenna WiFi network and
  the 433MHz/ELRS radios already are. A private local network is
  required by the rules, not just permitted; it isn't a "Wi-Fi" rule
  violation to be worried about.
- No tethers, wired links, fibre, or any cable connected to a drone
  during flight — confirms wireless-only for every link, no exceptions.
- **Single Ground Control Station, single unified operator interface —
  no separate GCS per drone.** The GCS must display, at minimum:
  mission status; **live camera feed from each drone**; position/
  estimated position of each drone; assigned search area/task per
  drone; detected + geotagged survivor locations; kit delivery status;
  comms/system health; consolidated mission progress.
- The "live camera feed from each drone" requirement, listed as a
  standing minimum-display item (not "on demand" or "switchable"),
  reads as simultaneous display of all 4 feeds on one interface — this
  is the concrete rulebook basis for going digital/WiFi video over a
  single-receiver analog approach, which physically can't show more
  than one feed at a time (see comms/ discussion history for why this
  ruled out the analog VTX route).

## Team / operations
- Team: 4-10 students + 1 faculty mentor (this team has 4, meets the
  minimum).
- Max 2 team members operating/supervising the Command & Control Station
  during the mission; only 1 of them may act as the operator at the GCS.
- One team member may be assigned to position/supervise each drone, and
  may reset/reload/reposition it where mission rules permit — no other
  team member may assist.

## Required fail-safes (minimum)
Return-to-home, communication-loss recovery, low-battery fail-safe,
geofence protection, mission-abort — all required per drone.

## Comms architecture (already decided — see README)
- Video + primary telemetry/commands: WiFi network (router + antenna)
- Backup telemetry: 433MHz radio, one pair per drone, distinct NetID each
- Manual override: ELRS, 1 transmitter (multi-model bind) + 1 receiver/
  drone — reserved for emergency recall/abort only, per the manual-
  intervention penalty above
