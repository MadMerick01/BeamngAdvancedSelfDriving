# BeamNG Advanced Self-Driving: Design

Status: draft, part 1 of the investigation (how the stock AI works today, BeamNG 0.39). Nothing here proposes changes yet.

Sources read: `lua/vehicle/ai.lua` (6,280 lines), `lua/ge/map.lua`, the `gameplay/traffic/` files (`vehicle.lua`, `roadTracking.lua`, `trafficUtils.lua`, `source.lua`, `baseRole.lua`, and roles `standard`, `police`, `suspect`, `empty`), plus the 0.39 GE API dump in `BolidesTheCutbyAbleMerick/docs/beamng-api/raw`.

---

## 1. The big picture

The AI has two layers that run in different Lua VMs.

| Layer | Where | Job |
|---|---|---|
| **Driver** | `lua/vehicle/ai.lua`, one copy per car, runs every graphics frame | Picks a route, builds a driving line, plans speed, and outputs steering, throttle, brake and handbrake |
| **Director** | `lua/ge/extensions/gameplay/traffic/*` and `gameplay_police`, one copy for the whole game, runs every 0.25 s | Spawns and respawns traffic, gives each car a role and personality, detects crashes and offences, and switches the driver's mode with `queueLuaCommand('ai.setMode(...)')` |

The director never steers. It only changes the driver's mode and settings. So anything about *how well a car drives* lives in `ai.lua`, and anything about *what the car decides to do* (pull over, chase, flee, give insurance) lives in the role files.

`map.lua` (GE side) builds the road graph (nodes with position, radius = half road width, and links with drivability, speed limit, one-way and lane strings) and ships it to each vehicle's `mapmgr`. The GE API dump only covers GE globals. The vehicle-side `ai.*` functions don't appear there, so `ai.lua` itself is the reference for those.

## 2. Driver modes (`ai.setMode`)

| Mode | What it does | How the route is chosen |
|---|---|---|
| `disabled` | AI off, restores the gearbox mode | none |
| `traffic` | Civilian driving: legal speed, stays in lane, obeys signals, gives way at junctions, overtakes, uses horn and hazards | `trafficPlan()` random legal path, with intersection logic in `trafficActions()` |
| `random` | Drives around aimlessly | `mapData:getRandomPath`, heavily biased to lanes if `driveInLane` is on |
| `span` | Tries to cover every road on the map | Picks the farthest edge it hasn't driven yet, `mapData:spanMap` |
| `manual` | Drives to a waypoint or along a given path or script. **Racing mode lives here** (`ai.setRacing(true)`) | `newManualPath()` shortest path, or scripted line |
| `chase` / `follow` | Drives to another vehicle. Follow keeps distance, chase rams | `chasePlan()` predicts the target's road. Falls back to driving straight at the target when off-road or close behind it |
| `flee` | Gets away from a vehicle | `fleePlan()`. Off-road it just drives directly away |
| `stop` | Brakes to a stop on its line, then switches itself to `disabled` | current route |
| `slotTraffic` | Follows an externally fed target position and speed (`setSlotTrafficTarget`) | none, it drives at the point |
| `script` | Replays a recorded trajectory (`scriptai`) | recorded |

Other driver settings: `setAggression` (0 to about 1.25, scales how much grip it uses), `setSpeedMode` (`off`, `limit`, `set`, `legal`), `driveInLane`, `setAvoidCars`, `setAggressionMode('rubberBand')` (on automatically for chase, follow and flee), `setCutOffDrivability`, `setRecoverOnCrash` (teleports after 60 s stuck), `setTractionModel`, `setSpeedProfileMode`, and `setParameters({...})` for about 20 tuning values (turn force, look-ahead, friction multiplier, which assists are on, and so on).

## 3. The driving pipeline, one frame

This all runs in `updateGFX` (around line 5178).

1. **Sense the ego car.** Front position, direction, velocity, and `staticFrictionCoef = 0.95 × obj:getStaticFrictionCoef()` (depends on surface and tyres). It never looks at the world except through the road graph, the bounding boxes of other cars, and a few static raycasts.
2. **Route.** A list of graph nodes (`path`), from the mode's planner above.
3. **Plan.** `planAhead()` turns the path into about 150 to 550 m of *plan nodes* (`getMinPlanLen`, enough to stop at 0.2 g). The nodes are densified near the car and carry lane limits that come from the link's lane string.
4. **Shape the line.** It's a spring and relaxation scheme, not an optimiser. Each node gets a "turn force" proportional to `1 − cos(angle between segments) × turnForceCoef`, which pushes it toward the inside of bends. In lane mode a weak 0.15 force pulls it toward the lane centre. Then it's clamped to road or lane width minus half the car width. Displacement is limited per frame (`springForceIntegratorDispLim = 0.1`). The result is a smoothed, corner-cutting line that converges over several frames.
5. **Other cars.** Vehicles near the plan are cached. Ones on a plan segment set a per-node `trafficSqVel` speed cap and a pass margin. Ones beside the car push the target point sideways (`side_avoidance`, a gap-deficit formula). In racing mode it also builds left and right offset plans and switches to one when it's faster *and* passes a grip check.
6. **Curvature.** Horizontal and vertical curvature per node, both temporally smoothed.
7. **Speed profile.** Max cornering speed per node = `sqrt(a_max / curvature)` with `a_max = min(aggression, μ) × g`, corrected for slope and crests. Then:
   - default (non-traffic): a single backward pass using a non-uniform circular motion formula
   - `traffic`: `'Back'` mode, a backward pass with a friction-circle model (`acc_eval_1` is linear and conservative, `acc_eval_2` is elliptical)
   - `'ForwBack'` adds a forward acceleration pass (not used by default)
   - after that, speed limit, set speed or manual speeds clip the result.
8. **Target point.** Look-ahead `max(speed × 0.6, 4.5 m)` along the line, smoothed between segments.
9. **Throttle and brake.** `throttleBrakeController` learns an online *throttle → acceleration* line fit while driving, then inverts it to reach the target acceleration. Brake is proportional to the shortfall.
10. **Steering and assists.** `driveToTarget()` steers by `asin(lateral offset of target / distance)`, the plain geometric angle to the look-ahead point, sent straight to `driveCar`. There's no wheelbase, slip angle or yaw-rate feedback in steering. Separately it scales throttle and brake by its own ABS, TCS, understeer and oversteer coefficients, computed from wheel slip and from how far the car is outside the line.
11. **Stuck handling.** Below about 0.1 m/s with a target to drive to, or with the target behind the car, it runs a **multi-point turn** routine (`twt`) using 8 corner raycasts and nearby bounding boxes. After 5 s with no movement it sets a crash manoeuvre flag. With `recoverOnCrash` on it teleports after 60 s.

## 4. Traffic director (GE side)

- **`vehicle.lua`** wraps each traffic car. It tracks position, collisions, damage and the respawn radius, and fades cars out and in to recycle them. It turns headlights on at night and in tunnels, runs the police pursuit state machine (offences → arrest or evade timers → roadblocks), and calls `setAiMode`, which also forces settings per mode (traffic: legal speed and lanes; chase and flee: aggression ≥ 0.8, lanes off).
- **`roadTracking.lua`** scores *every* vehicle, including the player, for speeding, wrong way and reckless driving, and detects red-light and stop-sign violations. Police use these scores.
- **Roles** (`baseRole` + `standard`, `police`, `suspect`, `empty`). Each car gets a personality of **aggression, patience and bravery**, each drawn from a Gaussian around 0.5. Today only aggression reaches the driver (±0.3). There's commented-out code to map patience to `trafficWaitTime`.
  - `standard`: reacts to crashes and near misses with pull over, follow, chase (road rage), flee, or exchange insurance, depending on bravery and patience.
  - `police`: follow when passive, chase when aggressive, dodges head-on hits by briefly switching to `flee`, sets roadblocks with `stop`, and pulls over next to a slowed suspect.
  - `suspect`: flee at aggression 0.8 with lanes and speed limits off.
- **`trafficUtils.lua`** finds spawn points (on the route ahead, then on a line, then radially) and snaps cars to a guessed lane. Lane count is estimated from road width (3.05 m lanes), because real lane data isn't used here yet.

## 5. Debug and diagnostic tools found

- **AI debug draw:** `ai.setVehicleDebugMode({debugMode = '<mode>'})` in the vehicle console. The drawing is done by `aiDebugDraw.lua`, which **I don't have yet**, so the full list of mode names isn't confirmed. `'target'` and `'all'` are confirmed in `ai.lua`. The traffic system also passes its `aiDebug` setting through, so `gameplay_traffic.setTrafficVars({aiDebug = '...'})` sets the mode on every traffic car.
- **Road graph view:** `map.toggleDrawNavGraph()` (GE console) draws the navgraph edges. `map.toggleDrawExcludeSet()` shows traffic exclusion zones.
- **Telemetry logging:** `ai.startStopDataLog('name')` toggles a per-frame CSV with position, speed, **target speed**, accelerations, velocities, throttle, brake, steering, friction coefficient and rpm. This is the best tool we have for measuring the current AI objectively.
- `ai.dumpParameters()` and `ai.dumpCurrentRoute()` print state. `gameplay_traffic.setDebugMode` and `gameplay_traffic_trafficUtils.debugMode` give spawn logging.

## 6. Weak spots the code itself admits

These come from comments and TODOs in the source, plus my reading of the code.

1. **Steering is purely geometric.** Steering is the angle to the look-ahead point with a fixed 0.6 s look-ahead. With no vehicle-dynamics term it's likely to weave at speed, cut corners on long cars, and handle trucks badly.
2. **The racing line isn't optimised.** The spring relaxation smooths corners but never minimises time or curvature over the whole corner. The comment at line 3693 says a different error smoother "behaves better for race driving".
3. **The speed profile is conservative in traffic** (`acc_eval_1`). The forward pass is off by default, and the stop-mode decel formula is flagged "check this calculation".
4. **Grip is one number** (`staticFrictionCoef × 0.95`). There's no load transfer, aero or per-vehicle GG diagram. The comment "has to be extracted from ggv" marks this as unfinished.
5. **Traffic overtaking is a hack.** The police role turns lanes off to overtake (`-- this is a hack; we should enable the vehicle AI to naturally overtake traffic in lane mode`), and pull-over "is not working properly … should be done in ai.lua".
6. **Crash avoidance lives in the role, not the driver.** Police dodging is a mode switch to `flee` for 3 s.
7. **Personality is mostly unused.** Patience and bravery only affect crash reactions, not driving style.
8. **Intersections** use positional checks every frame (the comment says "it would be great to improve this"), and signal selection ignores lanes.
9. **Perception** is the road graph plus other cars' bounding boxes plus a few static raycasts. Obstacles that aren't on the graph or aren't vehicles are mostly invisible except in the multi-point-turn routine.

## 7. Still to collect

| What | Why |
|---|---|
| `lua/vehicle/aiDebugDraw.lua` | Confirms every debug view and what it draws |
| `lua/vehicle/mapmgr.lua` and `lua/common/graphpath.lua` | Pathfinding (`getPath`, `getRandomPath`, `spanMap`) and how lane and drivability penalties work |
| `lua/vehicle/scriptai.lua` | Recorded-path driving and its own debug modes |
| `lua/ge/extensions/gameplay/traffic.lua` and `gameplay/police.lua` | The managers that drive the roles |
| `lua/ge/extensions/core/trafficSignals.lua` | Signal states that the intersection logic reads |
| In-game baseline runs | Same car and route in `manual`+racing, `traffic` and `chase`, with `startStopDataLog` on, at a few aggression values. This gives us numbers to compare against later. |

## 7b. Goals (from Rowan, 31 Oct)

1. **Now:** good car chases on ordinary town, city and countryside roads. That covers the `chase`, `follow` and `flee` modes plus police and suspect roles.
2. **Ultimately:** an AI that can drive anywhere on beamng.earth (the whole world). That means it must work on roads nobody has recorded, using only the road graph and live sensing.
3. **Later:** racing tracks (the per-track reference lap in section 8 below).

Confirmed by Rowan: stock traffic runs on beamng.earth, so it provides an AI road graph and the same `mapmgr` data the driver uses.

What this changes: because chases happen on arbitrary roads, **per-track replay isn't the main approach**. Rowan's recordings are used to learn a *general* driving model. Given road geometry ahead (curvature, width, gradient, crests, junctions, visibility) plus the chase situation (distance to the target and its speed), it predicts line, speed, brake and throttle like Rowan would. Phase 4b below becomes the core work. Phases 1 and 3 (recorder and tracking driver) are still needed. Phase 2 becomes "turn street-driving logs into training samples per corner".

Chase-specific work to investigate in `chasePlan()` and `fleePlan()`:
- how the target's road and route are predicted
- the rubber-band aggression
- the fallback of driving straight at the target when off-road or close behind it
- the police role's head-on dodge hack.

## 8. Goal: an AI that learns from Rowan's driving

Rowan is an experienced sim racer with a direct drive wheel and load cell pedals, which gives us high-quality analog steering, brake pressure and throttle data. The goal is to replace the stock racing line (spring relaxation) and the rudimentary throttle and brake control with behaviour learned from his laps.

### What already exists

`ai.startRecording()` / `ai.stopRecording()` (via `scriptai`) and `ai.startFollowing(script)`. If the script points carry `v`, it routes through `driveUsingPath({script = ...})` and follows the recorded line at the recorded speeds, but it still uses the stock steering and pedal controller (section 3, steps 9 and 10). This is our **baseline zero** to compare against.

### Phases

1. **Recorder (in-game Lua mod).** Log at about 100 Hz while Rowan drives: world position and heading, distance along the track, lateral offset from the centreline, speed, the vehicle's g-forces, yaw rate, wheel slip, gear, rpm, and raw *and* filtered steering, throttle, brake, clutch and handbrake. Store each lap as its own file, with car config, track and tyre metadata.
2. **Reference lap.** Pick the best lap, or merge the best few. Resample by track distance and extract:
   - the racing line, as lateral offset vs. distance
   - a speed profile
   - per-corner features: brake point, peak brake, trail-brake release rate, turn-in point, apex and minimum speed, throttle pickup point, and how fast throttle is applied on exit.
3. **Tracking driver.** A new controller, loaded as a mod that replaces or wraps `ai.lua`'s `driveToTarget` and `throttleBrakeController`:
   - **Steering:** feedforward from line curvature and wheelbase (with understeer correction), plus feedback on lateral and heading error and yaw rate, with look-ahead scheduled by speed.
   - **Pedals:** follow Rowan's brake and throttle profile against track distance, with closed-loop correction to the speed target and slip-based limits (trail-brake within the friction circle).
4. **Generalise.**
   - (a) Scale the profiles to a different car's grip and power, using a grip estimate measured from the logs.
   - (b) Learn a corner model (corner geometry → line, brake point and trail-brake shape) from many recorded corners, so tracks that haven't been recorded get human-like lines.
   - (c) Optionally compare with an offline minimum-curvature or minimum-time line.

### Metrics

Lap time vs. Rowan's lap, lateral deviation from the reference line, minimum speed per corner, brake point error, off-track count, and how smooth the steering and pedal inputs are. All of these come from the recorder, so stock AI, replay and the new driver can be compared on equal terms.

### Open questions

- First track and car to test on.
- Whether the mod overrides `ai.lua` (most control) or adds a new mode alongside it (safer across game updates).

## 9. Next sections

Architecture decision · Recorder file format · Milestones
