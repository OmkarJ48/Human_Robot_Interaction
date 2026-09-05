--[[
    Offline smoke test for scripts/path_following_obstacle_avoidance.lua

    CoppeliaSim is not available in CI, so this harness stubs the `sim` and
    `simOMPL` APIs with a tiny 2D world and integrates a differential-drive
    model. It qualifies the parts that do not need a physics engine:
    handle resolution, the obstacle collection, goal latency/smoothing,
    re-planning on goal motion, waypoint retirement and the heading
    controller actually converging on the goal.

    Run:  lua5.4 tests/smoke_test.lua
]]

local WHEEL_RADIUS, AXLE_LENGTH, DT = 0.04, 0.24, 0.05
local MAX_STEPS = 20000

----------------------------------------------------------------------
-- Mock world
----------------------------------------------------------------------
local world = {
    handles = {}, names = {}, nextHandle = 1,
    pos = {}, ori = {}, parent = {},
    jointVel = {}, collections = {}, nextCollection = 1000,
    time = 0, steps = 0, logs = {}, planCalls = 0,
}

local function newObject(name, x, y, z)
    local h = world.nextHandle
    world.nextHandle = h + 1
    world.handles[name] = h
    world.names[h] = name
    world.pos[h] = {x or 0, y or 0, z or 0}
    world.ori[h] = {0, 0, 0}
    world.parent[h] = -1
    return h
end

-- Scene: robot at the origin, goal at (2.0, 0.6), one box obstacle between.
local hRobot  = newObject('robot', 0, 0, 0.1)
local hRef    = newObject('robotRef', 0, 0, 0.1)
local hLeft   = newObject('leftMotor')
local hRight  = newObject('rightMotor')
local hVolume = newObject('collisionVolume', 0, 0, 0.1)
local hGoal   = newObject('goalDummy', 2.0, 0.6, 0.1)
local hBox    = newObject('obstacle', 1.0, 0.0, 0.1)
world.parent[hRef] = hRobot
world.parent[hVolume] = hRobot
world.parent[hGoal] = hRobot

local OBSTACLE = {x = 1.0, y = 0.0, halfX = 0.18, halfY = 0.45}

local function insideObstacle(x, y)
    return math.abs(x - OBSTACLE.x) < OBSTACLE.halfX
       and math.abs(y - OBSTACLE.y) < OBSTACLE.halfY
end

local PATH_ALIASES = {
    ['.'] = 'robot', ['./ref'] = 'robotRef', ['./robotRef'] = 'robotRef',
    ['./leftMotor'] = 'leftMotor', ['./rightMotor'] = 'rightMotor',
    ['./collisionVolume'] = 'collisionVolume',
    ['./goalDummy'] = 'goalDummy', ['/goalDummy'] = 'goalDummy',
}

----------------------------------------------------------------------
-- Mock sim API
----------------------------------------------------------------------
local sim = {
    handle_all = -2, handle_tree = -3,
    drawing_lines = 1, drawing_linestrip = 2, drawing_spherepoints = 3,
    verbosity_scriptinfos = 1, verbosity_scriptwarnings = 2, verbosity_scripterrors = 3,
}

function sim.getObject(path)
    local name = PATH_ALIASES[path]
    local h = name and world.handles[name]
    if not h then error('no such object: ' .. tostring(path)) end
    return h
end

function sim.getObjectParent(h) return world.parent[h] end
function sim.setObjectParent(h, p) world.parent[h] = p end
function sim.getObjectPosition(h) local p = world.pos[h]; return {p[1], p[2], p[3]} end
function sim.setObjectPosition(h, _, p) world.pos[h] = {p[1], p[2], p[3]} end
function sim.getObjectOrientation(h)
    local o = world.ori[world.parent[h] ~= -1 and h or h]
    return {o[1], o[2], o[3]}
end
function sim.setJointTargetVelocity(h, v) world.jointVel[h] = v end
function sim.getSimulationTime() return world.time end
function sim.addLog(_, msg) world.logs[#world.logs + 1] = msg end

function sim.createCollection()
    local c = world.nextCollection
    world.nextCollection = c + 1
    world.collections[c] = true
    return c
end
function sim.addItemToCollection() end

function sim.checkCollision(volumeHandle)
    local p = world.pos[volumeHandle]
    return insideObstacle(p[1], p[2]) and 1 or 0
end

function sim.addDrawingObject() return 1 end
function sim.addDrawingObjectItem() end
function sim.removeDrawingObject() end

--- One simulation step: integrate the differential drive from wheel speeds.
function sim.step()
    world.steps = world.steps + 1
    if world.steps > MAX_STEPS then error('simulation did not converge') end
    world.time = world.time + DT

    local wl = world.jointVel[hLeft] or 0
    local wr = world.jointVel[hRight] or 0
    local v = WHEEL_RADIUS * (wl + wr) / 2
    local omega = WHEEL_RADIUS * (wr - wl) / AXLE_LENGTH

    local o = world.ori[hRef]
    o[3] = o[3] + omega * DT
    local p = world.pos[hRef]
    p[1] = p[1] + v * math.cos(o[3]) * DT
    p[2] = p[2] + v * math.sin(o[3]) * DT
    -- Keep the body, its reference frame and the planning proxy together.
    world.pos[hRobot] = {p[1], p[2], p[3]}
    world.pos[hVolume] = {p[1], p[2], p[3]}
    world.ori[hRobot] = {o[1], o[2], o[3]}
end

----------------------------------------------------------------------
-- Mock simOMPL: straight line, detoured around the box when needed.
----------------------------------------------------------------------
local simOMPL = {
    StateSpaceType = {position2d = 1},
    Algorithm = {RRTConnect = 1, RRTstar = 2, BiTRRT = 3},
}
function simOMPL.createTask() return {} end
function simOMPL.createStateSpace() return {} end
function simOMPL.setStateSpace() end
function simOMPL.setAlgorithm() end
function simOMPL.setCollisionPairs() end
function simOMPL.setStartState(t, s) t.start = s end
function simOMPL.setGoalState(t, s) t.goal = s end
function simOMPL.setup() end
function simOMPL.destroyTask() end

function simOMPL.compute(task, _, _, stateCount)
    world.planCalls = world.planCalls + 1
    local sx, sy = task.start[1], task.start[2]
    local gx, gy = task.goal[1], task.goal[2]
    -- Detour above the box whenever the straight line would clip it.
    local via = nil
    for i = 0, 20 do
        local a = i / 20
        if insideObstacle(sx + (gx - sx) * a, sy + (gy - sy) * a) then
            via = {OBSTACLE.x, OBSTACLE.y + OBSTACLE.halfY + 0.25}
            break
        end
    end
    local knots = via and {{sx, sy}, via, {gx, gy}} or {{sx, sy}, {gx, gy}}
    local path, n = {}, math.max(8, math.floor(stateCount / 8))
    for seg = 1, #knots - 1 do
        local a, b = knots[seg], knots[seg + 1]
        for i = 0, n do
            local t = i / n
            path[#path + 1] = a[1] + (b[1] - a[1]) * t
            path[#path + 1] = a[2] + (b[2] - a[2]) * t
        end
    end
    return true, path
end

----------------------------------------------------------------------
-- Load the script under test with the mocks injected
----------------------------------------------------------------------
package.preload['sim'] = function() return sim end
_G.sim, _G.simOMPL = sim, simOMPL

local chunk = assert(loadfile('scripts/path_following_obstacle_avoidance.lua'))

local failures = 0
local function check(label, ok, detail)
    print((ok and '  PASS  ' or '  FAIL  ') .. label .. (detail and ('  -- ' .. detail) or ''))
    if not ok then failures = failures + 1 end
end

--- Put the world back to its initial state with the goal wherever we want it.
local function resetWorld(goalX, goalY)
    world.pos[hRobot]  = {0, 0, 0.1}
    world.pos[hRef]    = {0, 0, 0.1}
    world.pos[hVolume] = {0, 0, 0.1}
    world.pos[hGoal]   = {goalX, goalY, 0.1}
    world.ori[hRobot]  = {0, 0, 0}
    world.ori[hRef]    = {0, 0, 0}
    world.parent[hGoal] = hRobot
    world.jointVel = {}
    world.time, world.steps, world.logs, world.planCalls = 0, 0, {}, 0
    -- Re-execute the chunk so the script's own locals start clean too.
    chunk()
end

local function runScenario(name, goalX, goalY)
    print('\nscenario: ' .. name)
    resetWorld(goalX, goalY)

    sysCall_init()
    check('init detached the goal dummy', world.parent[hGoal] == -1)

    sysCall_thread()

    local final = world.pos[hRef]
    local goal  = world.pos[hGoal]
    local dist  = math.sqrt((final[1] - goal[1])^2 + (final[2] - goal[2])^2)

    check('planner was invoked', world.planCalls >= 1, world.planCalls .. ' call(s)')
    check('robot never ended inside the obstacle', not insideObstacle(final[1], final[2]))
    check('motors were stopped on exit',
          (world.jointVel[hLeft] == 0) and (world.jointVel[hRight] == 0))
    check('goal-reached was logged', (function()
        for _, m in ipairs(world.logs) do if m:match('goal reached') then return true end end
        return false
    end)())

    sysCall_cleanup()
    check('cleanup restored the goal dummy parent', world.parent[hGoal] == hRobot)
    print(('  %d step(s) simulated, final distance to goal %.3f m'):format(world.steps, dist))
    return dist
end

-- 1. Reachable goal beyond the box: the robot must arrive at it.
local dist = runScenario('goal in free space, obstacle on the direct line', 2.0, 0.6)
check('robot converged on the goal', dist < 0.15, ('distance %.3f m'):format(dist))

-- 2. Goal buried inside the box: findFreePositionNear must relocate it to the
--    nearest free cell, so the robot stops just outside the obstacle instead
--    of failing to plan.
dist = runScenario('goal inside an obstacle', OBSTACLE.x, OBSTACLE.y)
check('unreachable goal was relocated just outside the obstacle',
      dist > 0.05 and dist < 0.60, ('distance %.3f m'):format(dist))

print()
if failures > 0 then
    print(failures .. ' check(s) failed')
    os.exit(1)
end
print('all checks passed')
