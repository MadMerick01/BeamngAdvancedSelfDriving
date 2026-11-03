# Session wcusa_city_01: first look

8 chunks, 91,445 rows at about 213 Hz, 7.2 min total, 5.7 min moving (speed > 3 m/s). The logger v0.2 worked and every column is populated.

## How Rowan drove

| Metric | Value |
|---|---|
| Speed (median / 90% / max) | 73 / 109 / 147 km/h |
| Lateral g (90% / 99%) | 0.56 / 0.84 g |
| Braking g (median / 95% / 99%) | 0.43 / 0.70 / 0.75 g |
| Acceleration g (median / 99%) | 0.18 / 0.48 g |
| Combined g (99%) | 0.86 g, against a surface friction coefficient of about 1.10. So he used roughly 75–80% of the available grip, which is typical for fast road driving |
| Brake pedal (median / max) | 0.29 / **0.63**. He never pressed past 63% |
| Brake applications | 27 |
| Trail braking | 15% of braking time overlaps cornering above 0.3 g. Mostly he brakes in a straight line, then turns |
| Cornering on throttle vs coasting | 51 s vs 20 s. He powers through most corners |
| Gears used | 1 to 4 |
| ABS / TCS / ESC | all read 0. Either they're off or this car doesn't report them |
| Impacts | 3 spikes (above 2 g) at t ≈ 309, 335 and 376 s. They're excluded from the stats above only by the percentiles, and need trimming before training |

## Compared with the stock AI

- In `traffic` mode, aggression is about 0.3, and the stock speed planner caps *total* acceleration at `min(aggression, μ) × g`, which is about **0.3 g**. Rowan brakes at 0.7 g and corners at 0.84 g.
- `chase` and `flee` force aggression to at least 0.8, which gives about 0.8 g, close to Rowan's peaks. But the stock AI applies that one limit equally to braking, cornering and accelerating. Rowan's driving clearly isn't symmetric: he brakes hard, accelerates gently at 0.48 g because the car is traction-limited, and corners in between.
- **First concrete improvement:** give the speed planner separate braking, cornering and acceleration limits (a measured g-g envelope per car) instead of one aggression number.

## Logger issues found

- **Road matching at junctions:** the median heading error to the matched road was 3°, but the 90th percentile was 24° and 5% of samples sat outside the road edges. Near junctions `findClosestRoad` snapped to the crossing road. v0.3 uses the direction-aware `findBestRoad` instead.
- The brake max of 0.63 could be how Rowan drives, or a pedal calibration or deadzone issue. Worth checking.

## Session notes from Rowan
- Car: **Bruckell Moonhawk 309 V8, manual**. No ABS, no TC.
- Pace: a deliberate chase/flee pace, reasonably fast.
- **Braking was deliberately gentle so the heavy car stays balanced.** The 0.63 pedal max is technique, not calibration. So the braking limit for a heavy car without ABS is set by *stability* (weight transfer and rear lockup), not peak grip. The stock AI brakes up to its friction limit and has no idea about this. The model needs a braking limit per car that depends on stability.
- Manual gearbox: the stock AI forces the `arcade` gearbox, so shift points are useful data for a later gear-selection model, but they're not part of phase 1.
