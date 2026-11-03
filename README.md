# BeamNG Advanced Self-Driving

Making the BeamNG.drive (0.39) AI drive more like a skilled human, starting with car chases on town, city and country roads, and later anywhere on beamng.earth.

The approach: record a human driving (and chasing the AI) with a telemetry logger, analyse how they corner, brake and accelerate, and patch the stock vehicle `ai.lua` to match.

## What's here
- `design/design.md`: how the stock AI works, the goals, and the plan.
- `driveLogger/`: a vehicle mod that logs driving at 100 Hz to CSV, with wheel-button bindings for chase tests. See its README.
- `smartAI/`: the patched `ai.lua` mod (per-car cornering/braking/acceleration limits, flee route and stuck fixes). `build/patch_ai.py` rebuilds it from the stock 0.39 file. See its README.
- `tools/analyze.py`: turns a folder of logger CSVs into summary stats and a per-corner table.
- `logs/`: recorded sessions and their analyses.

## Install the mods
Copy `driveLogger/driveLogger.zip` and `smartAI/smartAI.zip` into your BeamNG `mods` folder and restart the game. smartAI replaces `lua/vehicle/ai.lua` and only works on 0.39.
