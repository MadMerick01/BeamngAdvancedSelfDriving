# chase_01: Rowan chasing smartAI v0.1 (flee), Moonhawk vs Moonhawk, West Coast USA streets

Both logs are about 2.2 km and about 110 s moving, at 85 Hz with no hitches (logger v0.6/0.7).

| | Rowan (chasing) | smartAI v0.1 (fleeing) |
|---|---|---|
| Speed p50 / p90 / max | 76 / 104 / 122 km/h | 71 / 112 / 130 |
| Lateral g p90 / p99 | 0.57 / 0.76 | 0.72 / 1.03 |
| Braking g p50 / p95 | 0.36 / 0.62 | **0.66 / 0.92** |
| Brake pedal p50 / max | 0.23 / 0.49 | **0.49 / 1.00** |
| Lateral g at 100–120 km/h (p98) | 0.33 | **1.20** |
| Impacts | 0 | 3 (start-up, kerb or slide at 92 s, pole at 122 s) |

## The pole (t ≈ 117–122 s)

The AI was at 121 km/h on full throttle through the start of a right-hander, where the road also narrows (half-width 6.6 m → 3.3 m). At 119 s it lifted abruptly. The rear stepped out (lateral velocity −3 m/s, slip about 8, countersteer +0.4), and it slid off the left edge at about 100 km/h into the pole.

## Causes and v0.2 fixes

1. **The cornering limit didn't depend on speed.** On streets Rowan uses about 0.8 g below 60 km/h but only 0.3–0.6 g above 80 km/h. The v0.1 AI used about 1 g at 120 km/h. Fixed with a speed-dependent `latTable`.
2. **Braking wasn't capped.** `brakeG = 0.65` only shaped the plan. The stock controller's brake grows with speed error (`decelerationCoef × speedDif`, unbounded), so it braked at up to 0.9 g with a full pedal. Fixed with a cap at 1.3 × brakeG.
3. **Not fixed yet:** abrupt throttle lift mid-corner (lift-off oversteer), steering that doesn't react to the slide, and the car-park circling in flee mode when it's off-road.
