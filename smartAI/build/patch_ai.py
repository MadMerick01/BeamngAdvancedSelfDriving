"""Builds smartAI's lua/vehicle/ai.lua from the stock 0.39 ai.lua.
Every replacement must match exactly once, so a game update that changes these lines fails loudly."""
import sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src).read()

def rep(old, new, count=1):
    global s
    n = s.count(old)
    assert n == count, f"expected {count} match(es), found {n}: {old[:80]!r}"
    s = s.replace(old, new)

# 1. smart profile state + elliptical brake/lateral adherence model
rep("""local acc_eval = acc_eval_2 -- Default adherence constraint
-- set the index for adherence constraint, useful only for Back or ForwBack mode
local function setTractionModel(model_index)
  if model_index == 1 then
    acc_eval = acc_eval_1
  else
    acc_eval = acc_eval_2
  end
end
""", """local acc_eval = acc_eval_2 -- Default adherence constraint

-- [[ smartAI: per-car limits learned from human driving logs ]] --
-- latG: cornering limit, brakeG: braking limit, accelG: traction limit (all in g, capped by surface friction)
-- tightRadius / tightLatScale: below this corner radius (m) the cornering limit is scaled down
-- `smart` (the active profile) is declared near the top of the file (v0.3)
-- latTable (v0.2): cornering limit vs speed {km/h, g}. On streets Rowan leaves much more margin at speed.
-- maxBrakeOver: controller may exceed brakeG by this factor only when the plan demands it (emergencies)
local smartProfiles = {
  moonhawk = {latG = 0.85, brakeG = 0.65, accelG = 0.45, tightRadius = 30, tightLatScale = 0.85, maxBrakeOver = 1.3, useAggression = false,
              latTable = {{0, 0.85}, {60, 0.85}, {80, 0.68}, {100, 0.58}, {130, 0.5}},
              source = 'Rowan street driving: wcusa_city_01 + chase_01 (Moonhawk 309 V8 manual)'},
  moonhawk_country = {latG = 0.95, brakeG = 0.65, accelG = 0.45, tightRadius = 30, tightLatScale = 0.85, maxBrakeOver = 1.3, useAggression = false,
              latTable = {{0, 0.9}, {40, 0.95}, {200, 0.95}},
              source = 'Rowan country roads: italy_01 (Moonhawk 309 V8 manual)'},
}

local function latFromTable(tbl, speed)
  local kmh = speed * 3.6
  if kmh <= tbl[1][1] then return tbl[1][2] end
  for i = 2, #tbl do
    if kmh <= tbl[i][1] then
      local a, b = tbl[i-1], tbl[i]
      return a[2] + (b[2] - a[2]) * (kmh - a[1]) / (b[1] - a[1])
    end
  end
  return tbl[#tbl][2]
end

-- cornering acceleration available for a given curvature, when the limit itself depends on speed
local function smartLatAccel(curv, scale, mu)
  local k = max(abs(curv), 1e-5)
  local tbl = smart.latTable
  local gLim = smart.latG
  if tbl then
    gLim = tbl[1][2]
    for _ = 1, 4 do -- fixed point: speed from limit, limit from speed
      local v = sqrt(min(gLim * scale, mu) * g / k)
      gLim = latFromTable(tbl, v)
    end
  end
  local a = min(gLim * scale, mu) * g
  if smart.tightRadius and k > 1 / smart.tightRadius then a = a * (smart.tightLatScale or 1) end
  return a
end
local stockAccEval = acc_eval

-- elliptical friction model with different longitudinal (braking) and lateral limits
local function acc_eval_smart(speedSq, acc_max, curvature, brk_max)
  local ay_used = speedSq * curvature
  if ay_used < acc_max then
    return (brk_max or acc_max) * sqrt(1 - square(ay_used / acc_max))
  else
    return 0
  end
end

local function smartScale()
  if smart and smart.useAggression then return clamp(aggression / 0.8, 0.4, 1.1) end
  return 1
end

-- set the index for adherence constraint, useful only for Back or ForwBack mode
local function setTractionModel(model_index)
  if model_index == 1 then
    acc_eval = acc_eval_1
  else
    acc_eval = acc_eval_2
  end
  stockAccEval = acc_eval
  if smart then acc_eval = acc_eval_smart end
end

local function setSmartProfile(p)
  if p == nil or p == false or p == 'off' then
    smart = nil
    acc_eval = stockAccEval
    print('[smartAI] OFF (stock speed planning)')
    if guihooks then guihooks.message('smartAI: OFF (stock AI)', 4, 'smartAI') end
    return
  end
  if type(p) == 'string' then
    if not smartProfiles[p] then print('[smartAI] unknown profile: ' .. p) return end
    p = smartProfiles[p]
  end
  smart = deepcopy(p)
  acc_eval = acc_eval_smart
  print(string.format('[smartAI] v0.3 ON latG=%.2f brakeG=%.2f accelG=%.2f', smart.latG, smart.brakeG, smart.accelG))
  if guihooks then guihooks.message(string.format('smartAI: ON (corner %.2fg, brake %.2fg, accel %.2fg)', smart.latG, smart.brakeG, smart.accelG), 4, 'smartAI') end
end
""")

# 2. solver calls pass the per-node braking limit (stock acc_eval ignores the extra arg)
rep("local ax_final = acc_eval(n1.speed, n1.acc_max, abs(n1.curvature))",
    "local ax_final = acc_eval(n1.speed, n1.acc_max, abs(n1.curvature), n1.brk_max)")
rep("local ax_possible_current = acc_eval(n2SpeedSq, n2.acc_max, abs(n2.curvature))",
    "local ax_possible_current = acc_eval(n2SpeedSq, n2.acc_max, abs(n2.curvature), n2.brk_max)")
rep("local ax_possible_next = acc_eval(vx_possible_next, n1.acc_max, abs(n1.curvature))",
    "local ax_possible_next = acc_eval(vx_possible_next, n1.acc_max, abs(n1.curvature), n1.brk_max)")
rep("ax_possible_next = acc_eval(vx_tmp, n1.acc_max, abs(n1.curvature))\n",
    "ax_possible_next = acc_eval(vx_tmp, n1.acc_max, abs(n1.curvature), n1.brk_max)\n")

# 3. planAhead speed planning: separate cornering and braking limits; smart forces the backward solver
rep("""  -- Speed Planning --
  local totalAccel = min(aggression, ego.staticFrictionCoef) * g

  local lastNode = plan[plan.planCount]""", """  -- Speed Planning --
  local totalAccel = min(aggression, ego.staticFrictionCoef) * g
  local latAccel, brkAccel = totalAccel, totalAccel
  if smart then
    local s = smartScale()
    latAccel = min(smart.latG * s, ego.staticFrictionCoef) * g
    brkAccel = min(smart.brakeG * s, ego.staticFrictionCoef) * g
    totalAccel = brkAccel
  end

  local lastNode = plan[plan.planCount]""")
rep("""  local gT = getTmpVec3(ego)
  if speedProfileMode then
    for i = 1, plan.planCount-1 do -- curvature is not defined on the last plan node
      local n1, n2 = plan[i], plan[i+1]
      -- consider inclination
      gT:setSub2(n2.pos, n1.pos); gT:setScaled(gravityDir:dot(gT) / max(square(n1.length), 1e-30)) -- gravity vec parallel to road segment: positive when downhill
      local gN = gravityDir:distance(gT) -- gravity component normal to road segment
      n1.acc_max = totalAccel * gN
      n1.speed""", """  local gT = getTmpVec3(ego)
  if speedProfileMode or smart then
    local tightCurv = smart and 1 / max(smart.tightRadius or 1, 1) or math.huge
    for i = 1, plan.planCount-1 do -- curvature is not defined on the last plan node
      local n1, n2 = plan[i], plan[i+1]
      -- consider inclination
      gT:setSub2(n2.pos, n1.pos); gT:setScaled(gravityDir:dot(gT) / max(square(n1.length), 1e-30)) -- gravity vec parallel to road segment: positive when downhill
      local gN = gravityDir:distance(gT) -- gravity component normal to road segment
      if smart then
        n1.acc_max = smartLatAccel(n1.curvature, smartScale(), ego.staticFrictionCoef) * gN
        n1.brk_max = brkAccel * gN
      else
        n1.acc_max = totalAccel * gN
        n1.brk_max = nil
      end
      n1.speed""")
rep("""    plan[plan.planCount].acc_max = plan[plan.planCount-1].acc_max

    if speedProfileMode == 'ForwBack' then
      solver_f_acc_profile(plan)
    end

    solver_b_acc_profile(plan)
  else -- Use standard algotihm with curvature""", """    plan[plan.planCount].acc_max = plan[plan.planCount-1].acc_max
    plan[plan.planCount].brk_max = plan[plan.planCount-1].brk_max

    if speedProfileMode == 'ForwBack' then
      solver_f_acc_profile(plan)
    end

    solver_b_acc_profile(plan)
  else -- Use standard algotihm with curvature""")

# 4. throttle/brake controller: separate accel and braking targets
rep("""  local acc_target = plan.targetAcc or min(aggression, ego.staticFrictionCoef) * g * accelCoef""",
"""  local accLim = min(aggression, ego.staticFrictionCoef) * g
  if smart then
    accLim = min((accelCoef > 0 and smart.accelG or smart.brakeG) * smartScale(), ego.staticFrictionCoef) * g
    -- v0.2: stock lets braking grow without bound with speed error; cap it near the braking limit
    accelCoef = max(accelCoef, -(smart.maxBrakeOver or 1.3))
  end
  local acc_target = plan.targetAcc or accLim * accelCoef""")

# 5. public API
rep("M.setTractionModel = setTractionModel\n", """M.setTractionModel = setTractionModel
M.setSmartProfile = setSmartProfile
M.getSmartProfile = function() return smart end
""")

# ---------------- v0.3: flee route, dead ends, stuck recovery ----------------
# 6. forward declarations (driveToTarget and updateGFX need them, and they sit above the smart block)
rep("""local lastCommand = {steering = 0, throttle = 0, brake = 0, parkingbrake = 0}
""", """local lastCommand = {steering = 0, throttle = 0, brake = 0, parkingbrake = 0}

-- [[ smartAI v0.3: forward declarations ]] --
local smart = nil -- active smartAI profile; nil means stock behaviour everywhere
-- stuck recovery state: time = seconds pushing without moving, force = seconds left of a forced escape,
-- dir = escape direction (1 forward, -1 reverse), tries within triesTimer seconds, gaveUp = parked until moved or reset
-- (v0.3 keeps its state in one table: the main chunk is close to Lua's 200-local limit)
local smartX = {stuck = {time = 0, force = 0, dir = 1, tries = 0, triesTimer = 0, gaveUp = false}}
function smartX.stuckReset()
  smartX.stuck.time, smartX.stuck.force, smartX.stuck.dir, smartX.stuck.tries, smartX.stuck.triesTimer, smartX.stuck.gaveUp = 0, 0, 1, 0, 0, false
end
internalState.smartX = smartX

function smartX.tick()
  if not smart then return end
  local st = smartX.stuck
  st.triesTimer = max(0, st.triesTimer - dt)
  if st.triesTimer == 0 then st.tries = 0 end
  if st.gaveUp then
    if ego.speed > 3 then st.gaveUp = false; st.tries = 0; st.time = 0 end -- something got us moving again
  elseif st.force > 0 then
    st.force = max(0, st.force - dt)
    if st.force == 0 then st.time = 0 end
  else
    local ev = electrics.values
    local reversing = (ev.gearIndex or 0) < 0 or ev.gear == 'R'
    -- arcade gearbox: in reverse the brake pedal drives the car
    local pushing = (reversing and lastCommand.brake > 0.25) or (not reversing and lastCommand.throttle > 0.25)
    if pushing and ego.speed < 0.4 and lastCommand.parkingbrake == 0 then
      st.time = st.time + dt
    else
      st.time = max(0, st.time - 2 * dt)
    end
    if st.time > 2.5 then
      st.time = 0
      st.tries = st.tries + 1
      st.triesTimer = 45
      twt.reset()
      if st.tries > 4 then
        st.gaveUp = true
        print('[smartAI] stuck: giving up after 4 tries')
        guihooks.message('smartAI: stuck, gave up (reset or move the car)', 6, 'smartAI')
      else
        st.dir = reversing and 1 or -1
        st.force = 2.5
        print('[smartAI] stuck: trying ' .. (st.dir > 0 and 'forward' or 'reverse') .. ' (try ' .. st.tries .. ')')
        guihooks.message('smartAI: stuck, trying ' .. (st.dir > 0 and 'forward' or 'reverse'), 3, 'smartAI')
      end
    end
  end
end
""")

# 7. stuck detection (updateGFX). Stock only counts throttle > 0.5, but the multi-point turn reverses with the
#    brake pedal (arcade gearbox), so a car spinning its wheels in reverse never counted as stuck (chase_03).
rep("""  if lastCommand.throttle > 0.5 and ego.speed < 1 then
    internalState.egoCannotMoveTime = internalState.egoCannotMoveTime + dt
  else
    internalState.egoCannotMoveTime = 0
  end
""", """  if lastCommand.throttle > 0.5 and ego.speed < 1 then
    internalState.egoCannotMoveTime = internalState.egoCannotMoveTime + dt
  else
    internalState.egoCannotMoveTime = 0
  end

  internalState.smartX.tick() -- smartAI v0.3 stuck detection (via internalState: updateGFX is at the upvalue limit)
""")

# 8. forced escape drive (driveToTarget), before the stock crash manoeuvre and multi-point turn
rep("""  if internalState.crash.maneuver == 1 and dirTarget < ego.dirVec:dot(internalState.crash.dir) then
""", """  if smart and smartX.stuck.gaveUp then
    twt.reset()
    driveCar(0, 0, 0, 1)
    return
  end
  if smart and smartX.stuck.force > 0 then
    -- drive the other way for a moment; gentle pedal first (less wheelspin), firmer on later tries
    twt.reset()
    local pedal = min(0.7, 0.3 + 0.1 * (smartX.stuck.tries - 1))
    if smartX.stuck.dir > 0 then
      driveCar(dirAngle, pedal, 0, 0)
    else
      driveCar(-dirAngle, 0, pedal, 0) -- arcade gearbox: brake pedal reverses from a stop
    end
    return
  end

  if internalState.crash.maneuver == 1 and dirTarget < ego.dirVec:dot(internalState.crash.dir) then
""")

# 9. clear stuck state on mode change
rep("""local function setMode(mode)
  if tableSizeC(wheels.wheels) == 0 then return end
""", """local function setMode(mode)
  if tableSizeC(wheels.wheels) == 0 then return end
  smartX.stuckReset()
""")

# 10. flee path that looks past each junction and avoids dead ends
rep("""local function fleePlan()
""", """-- [[ smartAI v0.3: flee route that avoids dead ends ]] --
-- Same greedy scoring as graphpath getFleePath, plus: at each junction ahead, a branch only counts as open if
-- the road network beyond it (not counting the way we came) reaches smartX.openRadius metres away.
smartX.openRadius = 300
smartX.openBudget = 600 -- nodes explored per check; a network bigger than this is assumed open
smartX.openCache, smartX.openCacheN, smartX.openCacheMap = {}, 0, nil
smartX.fleeDirCoef = {[false] = 1, [true] = 0.8}

function smartX.roadIsOpen(fromNode, child)
  if smartX.openCacheMap ~= mapData or smartX.openCacheN > 20000 then
    smartX.openCache, smartX.openCacheN, smartX.openCacheMap = {}, 0, mapData
  end
  local key = fromNode .. '\\0' .. child
  local c = smartX.openCache[key]
  if c ~= nil then return c end

  local graph, positions = mapData.graph, mapData.positions
  local origin = positions[child]
  local r2 = smartX.openRadius * smartX.openRadius
  local seen = {[fromNode] = true, [child] = true}
  local queue = {child}
  local head, open = 1, false
  while queue[head] do
    if head > smartX.openBudget then open = true break end
    local n = queue[head]
    head = head + 1
    if positions[n]:squaredDistance(origin) >= r2 then open = true break end
    for nb in pairs(graph[n] or {}) do
      if not seen[nb] then
        seen[nb] = true
        queue[#queue + 1] = nb
      end
    end
  end

  smartX.openCache[key] = open
  smartX.openCacheN = smartX.openCacheN + 1
  return open
end

function smartX.fleePath(startNode, initialDir, chasePos, pathLenLimit, rndDirCoef, rndDistCoef, checkFirst)
  local graph, graphpos = mapData.graph, mapData.positions
  if not graph or graph[startNode] == nil then return nil end
  pathLenLimit = pathLenLimit or 100
  rndDirCoef = rndDirCoef or 0
  rndDistCoef = min(rndDistCoef or 0.05, 1)
  local visited = {[startNode] = 0.2}
  local path = {startNode}
  local pathLen = 0
  local prevNode = startNode
  local prevDir = vec3(initialDir)
  local rnd2 = rndDirCoef * 2
  local chaseAIdist = graphpos[prevNode]:squaredDistance(chasePos) * 0.1
  repeat
    local maxScore, maxNode, maxVec, maxLen = -math.huge, -1, nil, nil
    local rDistCoef = min(1, pathLen * rndDistCoef)
    prevDir:set(prevDir.x + (math.random() * rnd2 - rndDirCoef) * rDistCoef, prevDir.y + (math.random() * rnd2 - rndDirCoef) * rDistCoef, 0)
    local prevPos = graphpos[prevNode]
    local chaseCoef = min(0.5, rndDistCoef * chaseAIdist)
    local links = graph[prevNode]
    local nLinks = 0
    for _ in pairs(links) do nLinks = nLinks + 1 end
    -- check junctions ahead; at the car's own start node only when extending a route (avoids needless U-turns)
    local checkDeadEnds = nLinks > 2 and (#path > 1 or checkFirst)
    for child, link in pairs(links) do
      local childPos = graphpos[child]
      local pathVec = childPos - prevPos
      local pathVecLen = pathVec:length()
      local vis = visited[child] or 1
      local score = vis * smartX.fleeDirCoef[link.oneWay and link.inNode == child] * link.drivability * (3 + pathVec:dot(prevDir) / max(pathVecLen, 1)) * max(0, 3 + (chasePos - childPos):normalized():dot(prevDir) * chaseCoef)
      if checkDeadEnds and score > 0 and not smartX.roadIsOpen(prevNode, child) then
        score = score * 0.02
      end
      visited[child] = vis * 0.2
      if score >= maxScore then
        maxNode, maxScore, maxVec, maxLen = child, score, pathVec, pathVecLen
      end
    end
    if maxNode == -1 then break end
    prevNode = maxNode
    prevDir = maxVec / (maxLen + 1e-30)
    pathLen = pathLen + maxLen
    table.insert(path, maxNode)
  until pathLen > pathLenLimit
  return path
end

function smartX.fleePathFn(startNode, dirVec, chasePos, len, a, b, checkFirst)
  if smart then return smartX.fleePath(startNode, dirVec, chasePos, max(len, getMinPlanLen(250)), a, b, checkFirst) end
  return mapData:getFleePath(startNode, dirVec, chasePos, len, a, b)
end

local function fleePlan()
""")

# 11. flee: extend the route before it runs out, even when the chaser is level with or ahead of us
rep("""    if ego.pos:dot(ego.dirVec) >= player.pos:dot(ego.dirVec) and not targetWPName and internalState.road ~= 'offroad' and plan.trafficMinProjSpeed > 3 then
      local path = currentRoute.path
      local pathCount = #path
      if pathCount >= 3 and plan[2].pathidx > pathCount * 0.7 then
        local cr1 = path[pathCount-1]
        local cr2 = path[pathCount]
        local dirVec = (push3(mapData.positions[cr2]) - mapData.positions[cr1]):normalized():copy()
        pathExtend(path, mapData:getFleePath(cr2, dirVec, player.pos, getMinPlanLen(), 0.01, 0.01))
        planAhead(currentRoute)
        return
      end
    end""", """    if (smart or ego.pos:dot(ego.dirVec) >= player.pos:dot(ego.dirVec)) and not targetWPName and internalState.road ~= 'offroad' and plan.trafficMinProjSpeed > 3 then
      local path = currentRoute.path
      local pathCount = #path
      if pathCount >= 3 and (plan[2].pathidx > pathCount * 0.7 or (smart and plan.planLen + getPathLen(path, plan[plan.planCount].pathidx) < getMinPlanLen(250))) then
        local cr1 = path[pathCount-1]
        local cr2 = path[pathCount]
        local dirVec = (push3(mapData.positions[cr2]) - mapData.positions[cr1]):normalized():copy()
        pathExtend(path, smartX.fleePathFn(cr2, dirVec, player.pos, getMinPlanLen(), 0.01, 0.01, true))
        if not smart or #path > pathCount then -- smart: if nothing could be added, fall through and replan
          planAhead(currentRoute)
          return
        end
        internalState.changePlanTimer = 0
      end
    end""")
rep("""      path = mapData:getFleePath(startnode, dirVec, player.pos, getMinPlanLen(), 0.01, 0.01)""",
    """      path = smartX.fleePathFn(startnode, dirVec, player.pos, getMinPlanLen(), 0.01, 0.01)""")

# 12. route ran out in flee: stock coasts straight with the brake on (the chase_02a crash at 140 km/h).
#     smart: follow the end of the road and ask for a new route.
rep("""local function fleePlan()
""", """-- returns true when it handled the end of the route (flee only)
function smartX.routeOut(plan)
  if not smart or M.mode ~= 'flee' then return false end
  plan.reRoute = true
  internalState.changePlanTimer = 0
  local pEnd, pPrev = plan[plan.planCount], plan[max(1, plan.planCount - 1)]
  local roadDir = pEnd.pos - pPrev.pos
  if roadDir:squaredLength() < 1e-4 then roadDir = vec3(ego.dirVec) end
  roadDir:normalize()
  driveToTarget(pEnd.pos + roadDir * max(15, ego.speed), 0, ego.speed > 20 and 0.1 or 0, max(5, ego.speed * 0.9))
  return true
end

local function fleePlan()
""")
rep("""    if not plan[egoSeg+2] and ((plan.targetSpeed < ego.lowTargetSpeedVal and ego.speed < 0.15) or (targetPos - ego.pos):dot(ego.dirVec) < 0) then
""", """    if not plan[egoSeg+2] and internalState.smartX.routeOut(plan) then return end

    if not plan[egoSeg+2] and ((plan.targetSpeed < ego.lowTargetSpeedVal and ego.speed < 0.15) or (targetPos - ego.pos):dot(ego.dirVec) < 0) then
""")

open(dst, "w").write(s)
print("patched OK ->", dst)
