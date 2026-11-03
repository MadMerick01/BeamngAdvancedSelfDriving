# smartAI v0.3 (BeamNG 0.39 only)

A patched copy of the stock `lua/vehicle/ai.lua`. Its main change: **speed planning and the throttle/brake targets use separate cornering, braking and acceleration limits** taken from your logged driving, instead of the single `aggression × grip` number. Since v0.3 it also fixes three flee problems from your chase tests (below). Steering and the racing line are still stock.

It's off by default. A car without `ai.setSmartProfile(...)` drives exactly like the stock AI.

## v0.3 changes (after chase_02a and chase_03)
All of these are only active when a smart profile is on, so the stock AI is untouched.
- **The flee route no longer runs out at speed.** In chase_02a the route ended at about 140 km/h; stock then holds the wheel straight with the brake on, and the car drifted off the road. Now the AI extends its route whenever less than about 250 m is left (even when you're alongside it), and if it still reaches the end it follows the road's direction and asks for a new route.
- **The flee route avoids dead ends.** At each junction ahead it checks whether the road beyond reaches at least 300 m away without coming back the way it came. Branches that don't are almost never picked. This is what sent it up the dead-end dirt road in chase_03.
- **Stuck recovery.** In chase_03 it reversed onto a rock and spun its wheels for 20+ s, because stock only counts "stuck" on the throttle and the turn-around reverses with the brake pedal. Now, if it pushes for 2.5 s without moving (either direction), it drives the other way for 2.5 s, gently first and a bit firmer on each retry. After 4 tries in 45 s it gives up, parks with the handbrake and shows "smartAI: stuck, gave up". It carries on by itself once it's moving again (for example after you push it).
- You'll see short on-screen messages like "smartAI: stuck, trying forward" when the recovery kicks in.

Not fixed yet: in a car park (off the road graph) flee still just drives directly away from you, which is what made it circle in your first test.

## v0.2 changes (after chase_01: the AI hit a pole at about 120 km/h)
- **The cornering limit now depends on speed.** On streets you corner at 0.7–0.85 g below 60 km/h but only 0.3–0.6 g above 80 km/h. v0.1 let the AI use 0.95 g at any speed, and it entered a narrowing right-hander at 121 km/h on full throttle, lifted, the rear stepped out, and it slid into the pole.
- **Braking is capped.** The stock controller's brake grows without limit as the speed error grows, so v0.1 still braked at up to 0.9 g and 100% pedal. Now it's held to 1.3× the braking limit (0.85 g).
- **Two profiles:** `'moonhawk'` (streets, default) and `'moonhawk_country'` (open roads, about 0.95 g at all speeds).

Corner speeds with the street profile: 20 m radius → 43 km/h, 40 m → 64, 80 m → 83, 150 m → 104, 300 m → 138.

## v0.1 Moonhawk profile (from wcusa_city_01 + italy_01)
| | stock flee (aggression 0.3–0.95) | smartAI |
|---|---|---|
| Cornering | 0.3–0.95 g | 0.95 g (× 0.85 in corners tighter than 30 m) |
| Braking | same as cornering | 0.65 g, gentler and earlier, like you |
| Acceleration | same as cornering | 0.45 g (traction-limited V8) |
| Rubber-banding | slows down when it gets far ahead of you | off, so it always runs at full pace |

All limits are capped by the surface grip.

## Install
1. Put `smartAI.zip` in your `mods` folder. Disable any other mod that replaces `ai.lua`.
2. Restart the game.

## Test: chase a fleeing smart Moonhawk
1. Load a map (West Coast USA or Italy), spawn **your** Moonhawk, then spawn a **second** Moonhawk. Keep only these two cars on the map, because flee automatically targets the only other car.
2. Get in the second car (Tab or Vehicle selector). Open the console, set the context to that vehicle, and run:
   ```
   ai.setSmartProfile('moonhawk')
   ai.setMode('flee')
   ```
   A message should say `smartAI: ON (corner 0.95g, brake 0.65g, accel 0.45g)`.
3. Switch back to your car (Tab). The AI keeps fleeing. Chase it.

**A/B against stock:** do the same with `ai.setSmartProfile('off')` instead, on the same roads.

**Tuning:** you can pass your own numbers, for example `ai.setSmartProfile({latG = 0.9, brakeG = 0.6, accelG = 0.45, tightRadius = 30, tightLatScale = 0.85})`.

**Optional:** log the AI car too. In the AI car's console, run `extensions.load('driveLogger')` then `driveLogger.start('ai_smart_flee')` before switching back. Then I can compare its corners against yours with the same tool.

## What to watch for
- v0.3: does it still pick dead ends? When it gets stuck, does it free itself? Tell me roughly where if not.
- Does it brake earlier and more smoothly into corners, and carry speed through flowing bends?
- Does it run wide or off the road anywhere? If so, cornering is too high there.
- Does it spin when accelerating out of corners? Steering and throttle are still stock, so expect some of that.

## Files
- `mod/lua/vehicle/ai.lua` is the patched file that goes in the zip.
- `build/patch_ai.py` rebuilds it from the stock 0.39 file and fails loudly if the game update changed the code it patches.
