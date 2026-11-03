# chase_02a + chase_03: smartAI v0.2 flee tests (Italy)

## chase_03: the long dirt-road chase (about 2 km, 115 s)

| | Rowan (chasing) | smartAI v0.2 (fleeing) |
|---|---|---|
| Speed p50 / p90 / max | 64 / 84 / 97 km/h | 63 / 83 / 107 |
| Lateral g p90 / p99 | 0.67 / 0.88 | 0.73 / 0.90 |
| Braking g p50 / p95 | 0.27 / 0.49 | 0.31 / 0.61 |
| Combined g p99 | 0.90 | 0.91 |
| Corners (median entry / min / exit) | 83 / 55 / 77 km/h (17 corners) | 83 / 54 / 80 km/h (22 corners) |
| Median peak brake pedal / g per corner | 0.26 / 0.45 g | 0.45 / 0.61 g |
| Median trail-brake per corner | 0.17 s | 0 s |
| Median apex position (fraction of half-width) | +0.40 | +0.04 |

**The overall pace and grip use match Rowan's closely.** The remaining differences:
- The AI brakes later and harder, then releases before turning in. Rowan brakes earlier and more gently, and trails into the corner.
- The AI apexes near the road centre, while Rowan stays on his side.

**Ending (t ≈ 116–146 s):** flee routed into a dead end. The AI started a multi-point turn, reversed at full lock onto a rock, and the car tilted (upZ 0.94, about 20°). It then sat in reverse at about 50% throttle for 20+ s with the wheels spinning (slip 40+) and never tried going forward.

## chase_02a: a short run before that (about 30 s)

At about 140 km/h on a near-straight, the AI's steering went to 0 and the brake held at a constant 0.3. That's the stock "end of route" behaviour (`driveCar(0, 0, max(0.3, brake))`), meaning flee's path ran out at speed. It drifted 5 m off the road edge and hit something at about 107 km/h. This is a stock flee routing bug, not the speed limits.

## Fixes needed (in priority order)
1. **Flee route runs out at speed (02a):** extend the path well before the end and never fall back to the steer-0/brake-0.3 branch above walking pace.
2. **Dead ends:** avoid dead-end branches when choosing the flee path. That needs `lua/common/graphpath.lua` (`getFleePath`).
3. **Stuck recovery:** if wheels are spinning with no movement for more than 3 s, switch direction. After repeated failures, give up cleanly.
4. **Braking style:** brake earlier and gentler and trail into the corner, plus keep to its own side (line work).
