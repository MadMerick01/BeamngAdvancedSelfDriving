# driveLogger (v0.7)

Records your driving to CSV so we can learn how you drive. It runs inside your car's vehicle Lua context and logs at 100 Hz.

## Install
Copy `driveLogger.zip` into `Documents/BeamNG.drive/<version>/mods/` (your user folder's `mods` directory) and restart the game, or press Ctrl+L to reload Lua.

## Easy way: one button
1. Go to **Options → Controls → Bindings** and search for `driveLogger`.
2. Bind **"driveLogger: start / stop recording"** to a wheel button or a key.
3. Press it once to start recording and again to stop. A message pops up on screen each time.

Optional: to name your sessions, open the console once (vehicle context) and run `driveLogger.setName('italy_country')`. Without a name, files are named by date and time.

## Console way
1. Spawn your car and open the console with `~`. Set the context dropdown to **your vehicle**, not GE.
2. `extensions.load('driveLogger')`
3. `driveLogger.start('wcusa_city')` (pick a name that says map + area)
4. Drive. Every 30 s a chunk file is written.
5. `driveLogger.stop()` when done. `driveLogger.status()` shows whether it's running and whether the map has a road graph.

Repeat step 2 after you switch or respawn the car.

The files are named `driveLogger_<name>_000.csv`, `_001.csv`, …, and should appear in your user folder (the same place `ai.startStopDataLog` writes to). Upload them here.

## What's logged
- Time, position, heading, and speeds (along the car, sideways, up), plus yaw rate and g-forces
- Raw inputs (steering, throttle, brake, clutch, handbrake) and filtered inputs, gear, rpm and wheel speed
- Wheel slip (average, driven wheels, max), wheels in the air, surface friction, and whether ABS, TCS or ESC are active
- Road context at 10 Hz: nearest road graph nodes, distance from the centre line, lateral offset (+ = right), road half-width, and heading error to the road

## Tips for good data
- Drive like you would in a chase: quick, but clean and consistent.
- Mix city blocks, tight town streets, and fast flowing country roads.
- Tell me the car and its config and whether you used ABS, TCS or ESC.
- On beamng.earth, run `driveLogger.status()` first. If it says `NO ROAD GRAPH`, the logger still records everything except the road columns, and we can work out road geometry from your own path.

## AI chase test (v0.7): two buttons
Bind these in **Options → Controls → Bindings** (search `driveLogger`):
- **"AI test - smart flee + log (this car)"** turns the car you're in into a fleeing smartAI Moonhawk and starts its logger. It needs `smartAI.zip` installed.
- **"AI test - stock flee + log (this car)"** does the same with the stock AI, for comparison. Binding it is optional.

Steps (keep only two cars on the map, because flee targets the other car):
1. Sit in the AI Moonhawk and press the **AI test** button. It starts fleeing straight away.
2. Switch to your car (Tab) and press your **start/stop recording** button.
3. Chase it for a few minutes.
4. Press **start/stop** to stop your log. Then Tab to the AI car and press **AI test** again to stop its log and bring it to a stop.

If you skip step 4 for the AI car, everything up to its last 30 s chunk is still saved.

## Resetting a test (v0.7)
- **"STOP ALL"** button: from any car, this stops every logger and brings the AI car to a stop. Logs are saved.
- To move the cars, use the free camera and **Vehicle → Teleport to camera**, or just drive them out yourself. The AI car is stopped, so you can get in and drive it.
- Start a new test the usual way: sit in the AI car and press **AI test**, then go to your car and press **record**.
- The **AI test** and **record** buttons also still toggle on their own if you press them in that car.
