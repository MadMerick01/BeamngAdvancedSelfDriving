# Session italy_01: Italy country roads, Moonhawk 309 V8 manual, no ABS/TC, traffic off

Made with `python3 tools/analyze.py logs/italy_01`. The per-corner table is in `corners.csv`.

## Summary (city session for comparison)

| | Italy | City (wcusa_city_01) |
|---|---|---|
| Moving time / distance | 476 s / 11.5 km | 344 s / 7.2 km |
| Speed p50 / p90 / max | 82 / 136 / 159 km/h | 73 / 109 / 147 |
| Lateral g p90 / p99 | **0.82 / 1.20** | 0.56 / 0.84 |
| Braking g p50 / p95 / p99 | 0.37 / 0.63 / 0.71 | 0.43 / 0.70 / 0.75 |
| Accel g p50 / p99 | 0.14 / 0.41 | 0.18 / 0.48 |
| Combined g p99 (μ ≈ 1.10) | 1.22 | 0.86 |
| Brake pedal p50 / max | 0.24 / 0.60 | 0.29 / 0.63 |
| Trail-brake share of braking time | **44%** | 15% |
| Cornering on throttle / coasting | 176 s / 51 s | 51 s / 20 s |
| Elevation range | 82 m | 44 m |
| Impacts | 1 | 7 spike samples |

## Corners: 69 detected

- **Angle:** 26 were gentle (20–45°), 18 medium (45–70°), 12 near 90°, and 11 tight or hairpin (>110°).
- **Radius:** 8 under 30 m, 10 between 30 and 60 m, 27 between 60 and 120 m, and 24 over 120 m.
- **Technique:** 45 braked, 10 lifted only, 14 flat out.
- **Median corner:** entry 96 km/h, min 68 km/h, exit 83 km/h, peak lateral g 1.03.
- **Line:** the median apex position is +0.13 of the road half-width (right of centre, on Rowan's side). The 5th to 95th percentile spans −0.8 to +1.0, so he uses the full road when he can see it's clear.

| Radius | n | Median min speed | Median peak lat g | Median peak brake |
|---|---|---|---|---|
| < 30 m | 8 | 27 km/h | 0.86 | 0.44 |
| 30–60 m | 10 | 42 km/h | 1.23 | 0.36 |
| 60–120 m | 27 | 68 km/h | 1.09 | 0.36 |
| > 120 m | 24 | 97 km/h | 0.92 | 0.12 |

## What this tells the model

1. **The limits aren't the same in every direction, and that's confirmed.** Rowan corners right at the grip limit (about 1.0–1.2 g), but brakes at only about 0.4–0.7 g and accelerates at about 0.4 g. The stock AI's single `aggression × μ × g` limit can't represent that.
2. **Braking depends on the road.** On flowing roads he trails the brake into the corner much more (44% vs 15% in the city).
3. **The tightest corners are taken more slowly than grip allows** (0.86 g in corners under 30 m). This is probably about traction and rotation in a heavy V8 at low speed. A good model should learn this rather than use pure friction-circle speeds.
4. **About a third of corners need no brake at all** (lift or flat). The stock AI brakes whenever its speed plan says so, so this is a quick place to make it feel more natural.

## Known analysis limits (v1 tool)

- `brake_dist_before_apex_m` (median 184 m) is unreliable. The 250 m look-back often catches braking for the previous corner. It needs splitting by braking event, not by distance window.
- The radius is a median over the turning phase, so it's rough. Peak lateral g above about 1.3 comes from bumps, slides or the impact, not steady cornering.
