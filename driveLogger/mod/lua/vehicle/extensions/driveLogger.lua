-- driveLogger: records a human driver's inputs, vehicle state and road context
-- for training a better BeamNG AI driver. Vehicle Lua context.
--
-- Usage (vehicle console, i.e. the console context set to your car):
--   extensions.load('driveLogger')
--   driveLogger.start('wcusa_city')   -- session name, used in the file names
--   driveLogger.stop()                -- flushes the last chunk
--   driveLogger.status()
--
-- Writes one CSV per chunk (default 60 s) through BeamNG's csvlib, the same
-- writer ai.lua uses for ai.startStopDataLog().

local M = {}

local CHUNK_SECONDS = 30 -- v0.7: smaller, more frequent writes (smaller hitch)
local SAMPLE_INTERVAL = 0.01 -- v0.7: log at 100 Hz instead of every frame
local ROAD_UPDATE_INTERVAL = 0.1 -- road lookups at 10 Hz (kd-tree query)

local HEADERS = {
  "t", "dt",
  "posX", "posY", "posZ", "dirX", "dirY", "dirZ", "upZ",
  "speed", "vLong", "vLat", "vUp", "yawRate",
  "accLong", "accLat", "accUp",
  "steeringInput", "throttleInput", "brakeInput", "clutchInput", "parkingbrakeInput",
  "steering", "throttle", "brake",
  "gear", "rpm", "wheelspeed",
  "slipAvg", "slipMaxDriven", "slipMax", "wheelsInAir", "frictionCoef",
  "roadN1", "roadN2", "roadDist", "roadHalfWidth", "roadLatOffset", "roadXnorm", "roadHeadingErr",
  "absActive", "tcsActive", "escActive",
}

local csvlib
local state = {
  running = false, name = nil, chunkIdx = 0, chunkTime = 0, t = 0, rows = 0, csv = nil,
  roadTimer = 0, road = {n1 = "", n2 = "", dist = -1, halfWidth = -1, lat = 0, xnorm = 0, headErr = 0},
  prevDir = nil,
}

local function newChunk()
  state.csv = csvlib.newCSV(unpack(HEADERS))
  state.chunkTime = 0
end

local function flushChunk()
  if state.csv and state.rows > 0 then
    local fname = string.format("driveLogger_%s_%03d", state.name, state.chunkIdx)
    state.csv:write(fname)
    print("[driveLogger] wrote " .. fname)
    state.chunkIdx = state.chunkIdx + 1
  end
  state.csv = nil
  state.rows = 0
end

local function ev(key) -- electrics value or 0
  local v = electrics.values[key]
  if type(v) == "boolean" then return v and 1 or 0 end
  return tonumber(v) or 0
end

local function updateRoad(pos, dirVec)
  local r = state.road
  local mapData = mapmgr.mapData
  if not mapData or not (mapmgr.findBestRoad or mapmgr.findClosestRoad) then r.n1, r.n2, r.dist = "", "", -1 return end
  -- v0.7: direction-aware lookup, so junctions don't snap to the crossing road
  local n1, n2
  if mapmgr.findBestRoad then n1, n2 = mapmgr.findBestRoad(pos, dirVec) end
  if not n1 and mapmgr.findClosestRoad then n1, n2 = mapmgr.findClosestRoad(pos) end
  if not n1 or not n2 then r.n1, r.n2, r.dist = "", "", -1 return end
  local p1, p2 = mapData.positions[n1], mapData.positions[n2]
  if not p1 or not p2 then return end
  -- orient the segment along the direction of travel
  if (p2 - p1):dot(dirVec) < 0 then n1, n2, p1, p2 = n2, n1, p2, p1 end
  local xnorm = clamp(pos:xnormOnLine(p1, p2), 0, 1)
  local onLine = p1 + (p2 - p1) * xnorm
  local segDir = (p2 - p1); segDir.z = 0; segDir:normalize()
  local right = vec3(segDir.y, -segDir.x, 0)
  local offset = pos - onLine; offset.z = 0
  local rad1, rad2 = mapData.radius[n1] or 0, mapData.radius[n2] or 0
  local flatDir = vec3(dirVec.x, dirVec.y, 0); flatDir:normalize()
  r.n1, r.n2 = tostring(n1), tostring(n2)
  r.dist = offset:length()
  r.lat = offset:dot(right) -- + = right of road centre line
  r.halfWidth = rad1 + (rad2 - rad1) * xnorm
  r.xnorm = xnorm
  r.headErr = math.atan2(segDir:cross(flatDir).z, segDir:dot(flatDir)) -- rad, + = pointing left
end

local function wheelStats()
  local total, totalDown, maxDriven, maxAll, inAir = 0, 0, 0, 0, 0
  local lwheels = wheels.wheels
  for i = 0, tableSizeC(lwheels) - 1 do
    local wd = lwheels[i]
    if wd and not wd.isBroken then
      local slip, down = wd.lastSlip or 0, wd.downForceRaw or 0
      total = total + slip * down
      totalDown = totalDown + down
      if wd.isPropulsed then maxDriven = math.max(maxDriven, slip) end
      maxAll = math.max(maxAll, slip)
      if down <= 1 then inAir = inAir + 1 end
    end
  end
  return total / (totalDown + 1e-25), maxDriven, maxAll, inAir
end

local function updateGFX(dt)
  if not state.running or dt <= 0 then return end
  state.t = state.t + dt
  state.chunkTime = state.chunkTime + dt
  state.sampleAcc = (state.sampleAcc or 0) + dt
  if state.sampleAcc < SAMPLE_INTERVAL then return end
  dt = state.sampleAcc -- time since the previous logged sample
  state.sampleAcc = 0

  local pos = obj:getPosition()
  local dirVec = obj:getDirectionVector()
  local upVec = obj:getDirectionVectorUp()
  local vel = vec3(obj:getSmoothRefVelocityXYZ())
  local rightVec = dirVec:cross(upVec); rightVec:normalize()

  local yawRate = 0
  if state.prevDir then
    local c = state.prevDir:cross(dirVec)
    yawRate = math.atan2(c:dot(upVec), state.prevDir:dot(dirVec)) / dt
  end
  state.prevDir = vec3(dirVec)

  state.roadTimer = state.roadTimer - dt
  if state.roadTimer <= 0 then
    state.roadTimer = ROAD_UPDATE_INTERVAL
    updateRoad(pos, dirVec)
  end
  local r = state.road
  local slipAvg, slipDriven, slipMax, inAir = wheelStats()

  state.csv:add(
    state.t, dt,
    pos.x, pos.y, pos.z, dirVec.x, dirVec.y, dirVec.z, upVec.z,
    vel:length(), vel:dot(dirVec), vel:dot(rightVec), vel:dot(upVec), yawRate,
    -sensors.gy, sensors.gx, sensors.gz,
    ev("steering_input"), ev("throttle_input"), ev("brake_input"), ev("clutch_input"), ev("parkingbrake_input"),
    ev("steering"), ev("throttle"), ev("brake"),
    ev("gear"), ev("rpm"), ev("wheelspeed"),
    slipAvg, slipDriven, slipMax, inAir, obj:getStaticFrictionCoef(),
    r.n1, r.n2, r.dist, r.halfWidth, r.lat, r.xnorm, r.headErr,
    ev("absActive"), ev("tcsActive"), ev("escActive")
  )
  state.rows = state.rows + 1

  if state.chunkTime >= CHUNK_SECONDS then
    flushChunk()
    newChunk()
  end
end

local function start(name)
  if state.running then print("[driveLogger] already running") return end
  local ok, lib = pcall(require, "csvlib")
  if not ok or not lib then print("[driveLogger] csvlib not available: " .. tostring(lib)) return end
  csvlib = lib
  if mapmgr and mapmgr.requestMap and not mapmgr.mapData then mapmgr.requestMap() end
  state.name = tostring(name or os.date("%Y%m%d_%H%M%S"))
  state.chunkIdx, state.t, state.rows, state.roadTimer, state.prevDir = 0, 0, 0, 0, nil
  newChunk()
  state.running = true
  print("[driveLogger] v0.7 started session " .. state.name .. " (run driveLogger.status() after a few seconds; t and rows should be rising)")
end

local function stop()
  if not state.running then print("[driveLogger] not running") return end
  state.running = false
  flushChunk()
  print(string.format("[driveLogger] stopped, %.1f s logged in %d chunk(s)", state.t, state.chunkIdx))
end

local defaultName = nil
local function msg(text)
  print("[driveLogger] " .. text)
  if guihooks and guihooks.message then guihooks.message("driveLogger: " .. text, 4, "driveLogger") end
end

local function setName(name) defaultName = name and tostring(name) or nil; msg("next session name: " .. tostring(defaultName)) end

local function toggle()
  if state.running then
    stop()
    msg(string.format("STOPPED, %.0f s saved in %d file(s)", state.t, state.chunkIdx))
  else
    start(defaultName and (defaultName .. "_" .. os.date("%H%M%S")) or nil)
    if state.running then msg("RECORDING " .. state.name) end
  end
end

-- AI test: turns THIS car into a fleeing AI (smartAI or stock) and logs it. Press again to stop.
local function aiTest(kind)
  kind = kind or 'smart'
  if state.running then
    stop()
    if ai and ai.setMode then ai.setMode('stop') end
    msg(string.format("AI test STOPPED, %.0f s saved in %d file(s)", state.t, state.chunkIdx))
    return
  end
  if not ai or not ai.setMode then msg("vehicle AI not available") return end
  if kind == 'smart' then
    if not ai.setSmartProfile then msg("smartAI mod not installed - install smartAI.zip") return end
    ai.setSmartProfile('moonhawk')
  elseif ai.setSmartProfile then
    ai.setSmartProfile('off')
  end
  start('ai_' .. kind .. '_flee_' .. os.date("%H%M%S"))
  if not state.running then return end
  ai.setMode('flee')
  msg("AI " .. string.upper(kind) .. " FLEE + logging. Switch to your car now!")
end

-- stop everything on this car: logger off, and AI brought to a stop if it is driving
local function halt()
  local wasLogging = state.running
  if state.running then stop() end
  if ai and ai.mode and ai.mode ~= 'disabled' then ai.setMode('stop') end
  if wasLogging then msg("halted, log saved") end
end

local function status()
  print(string.format("[driveLogger] v0.7 running=%s name=%s t=%.1f rows=%d chunks=%d road=%s",
    tostring(state.running), tostring(state.name), state.t, state.rows, state.chunkIdx,
    mapmgr.mapData and "yes" or "NO ROAD GRAPH"))
end

local function onReset() state.prevDir = nil end

M.start = start
M.stop = stop
M.status = status
M.toggle = toggle
M.aiTest = aiTest
M.halt = halt
M.setName = setName
M.onReset = onReset
-- registered once at load: the extension system caches hook functions when
-- the extension loads, so swapping M.updateGFX later is never picked up (v0.1 bug)
M.updateGFX = updateGFX

return M
