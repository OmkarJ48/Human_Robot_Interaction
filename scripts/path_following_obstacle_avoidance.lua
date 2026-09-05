--[[
    Human-Robot Interaction -- Maze navigation with OMPL path planning
    ------------------------------------------------------------------
    CoppeliaSim (4.x) threaded child script.

    Attach this script to the mobile robot inside
    scenes/Human_Robot_Interaction_Assignment_Simulation1.ttt.

    The robot plans a collision-free route through a maze of obstacles and
    walls, follows it with a differential-drive controller, and re-plans
    whenever the goal dummy is moved during the simulation.

    Pipeline (see docs/Human_Robot_Interaction_Assignment_Simulation_Report.pdf):
      1. sysCall_init   -- resolve handles, build the obstacle collection, set params
      2. sysCall_thread -- find a collision-free start, sanitise the goal,
                           plan with OMPL, execute and re-plan on goal motion
      3. sysCall_cleanup-- re-parent the goal dummy and clear the drawing objects
]]

local sim   = require('sim')
local simUI = nil            -- optional, not required for headless runs

----------------------------------------------------------------------
-- Tunable parameters
----------------------------------------------------------------------
local P = {
    -- Motion
    maxWheelSpeed      = 6.0,   -- rad/s, saturation for both wheels
    cruiseSpeed        = 4.0,   -- rad/s, nominal forward speed
    headingGain        = 2.5,   -- P gain, heading error -> differential speed
    waypointTolerance  = 0.06,  -- m, distance at which a waypoint is retired
    goalTolerance      = 0.08,  -- m, distance at which the goal is reached

    -- Planning
    plannerName        = 'RRTConnect', -- OMPL algorithm
    planningTime       = 4.0,   -- s, per planning attempt
    planningAttempts   = 6,     -- retries before giving up
    pathStateCount     = 240,   -- states requested when serialising the path
    searchRange        = 3.0,   -- m, half-extent of the 2D state space

    -- Goal handling
    goalLatency        = 0.25,  -- s, simulated perception latency
    goalSmoothing      = 0.30,  -- 0..1, low-pass factor on the tracked goal
    goalMoveThreshold  = 0.15,  -- m, goal displacement that triggers a re-plan
    goalSearchStep     = 0.05,  -- m, spiral step when the goal is inside an obstacle
    goalSearchMaxRings = 40,    -- how far the spiral search may wander

    -- Visualisation
    drawPath           = true,
    drawTrace          = true,
}

----------------------------------------------------------------------
-- Handles and state
----------------------------------------------------------------------
local robot, robotRef, goalDummy, goalParent
local leftMotor, rightMotor
local collisionVolume            -- convex proxy used for planning queries
local obstaclesCollection        -- collection handle of everything to avoid
local collisionPairs             -- {collisionVolume, obstaclesCollection}
local pathLineCont, traceLineCont, nodeDrawCont

local goalBuffer = {}            -- {t, position} samples for the latency model
local trackedGoal = nil          -- smoothed, latency-delayed goal position

--- Resolve an object by any of several candidate paths.
-- Keeps the script usable when the scene uses slightly different names.
local function resolveObject(candidates, optional)
    for _, name in ipairs(candidates) do
        local ok, handle = pcall(sim.getObject, name)
        if ok and handle and handle ~= -1 then
            return handle
        end
    end
    if not optional then
        error('Could not resolve any of: ' .. table.concat(candidates, ', '))
    end
    return -1
end

function sysCall_init()
    -- Reset per-run state: the script object survives between simulation
    -- runs, so a stale tracked goal would otherwise leak into the next run.
    goalBuffer, trackedGoal = {}, nil

    robot     = sim.getObject('.')
    robotRef  = resolveObject({'./ref', './robotRef', '.'}, true)
    if robotRef == -1 then robotRef = robot end

    leftMotor  = resolveObject({'./leftMotor',  './leftJoint',  '../leftMotor'})
    rightMotor = resolveObject({'./rightMotor', './rightJoint', '../rightMotor'})

    -- Convex volume swept for collision queries. Falls back to the robot
    -- body itself if the scene has no dedicated planning proxy.
    collisionVolume = resolveObject({'./collisionVolume', './collVolume'}, true)
    if collisionVolume == -1 then collisionVolume = robot end

    goalDummy  = resolveObject({'./goalDummy', '/goalDummy', './target', '/target'})
    goalParent = sim.getObjectParent(goalDummy)
    -- Detach so the goal can be dragged independently while the robot moves.
    sim.setObjectParent(goalDummy, -1, true)

    obstaclesCollection = sim.createCollection(0)
    sim.addItemToCollection(obstaclesCollection, sim.handle_all, -1, 0)
    -- Everything below the robot is "self", never an obstacle.
    sim.addItemToCollection(obstaclesCollection, sim.handle_tree, robot, 1)
    sim.addItemToCollection(obstaclesCollection, sim.handle_tree, goalDummy, 1)
    collisionPairs = {collisionVolume, obstaclesCollection}

    pathLineCont  = sim.addDrawingObject(sim.drawing_lines, 3, 0, -1, 10000, {0.2, 0.8, 0.2})
    traceLineCont = sim.addDrawingObject(sim.drawing_linestrip, 2, 0, -1, 10000, {0.2, 0.4, 1.0})
    nodeDrawCont  = sim.addDrawingObject(sim.drawing_spherepoints, 0.02, 0, -1, 10000, {1.0, 0.6, 0.1})

    sim.setJointTargetVelocity(leftMotor, 0)
    sim.setJointTargetVelocity(rightMotor, 0)
end

function sysCall_cleanup()
    if leftMotor and leftMotor ~= -1 then sim.setJointTargetVelocity(leftMotor, 0) end
    if rightMotor and rightMotor ~= -1 then sim.setJointTargetVelocity(rightMotor, 0) end
    -- Restore the scene hierarchy exactly as it was found.
    if goalDummy and goalParent then
        pcall(sim.setObjectParent, goalDummy, goalParent, true)
    end
    for _, c in ipairs({pathLineCont, traceLineCont, nodeDrawCont}) do
        if c then pcall(sim.removeDrawingObject, c) end
    end
end

----------------------------------------------------------------------
-- Collision checking
----------------------------------------------------------------------

--- Is the robot's collision volume free at `pos`?
-- The volume is teleported, tested, and restored, so the query never
-- disturbs the dynamic simulation.
local function checkCollidesAt(pos)
    local saved = sim.getObjectPosition(collisionVolume, -1)
    sim.setObjectPosition(collisionVolume, -1, {pos[1], pos[2], saved[3]})
    local hit = sim.checkCollision(collisionVolume, obstaclesCollection)
    sim.setObjectPosition(collisionVolume, -1, saved)
    return hit > 0
end

--- Spiral outwards from `pos` until a collision-free position is found.
local function findFreePositionNear(pos)
    if not checkCollidesAt(pos) then return pos end
    for ring = 1, P.goalSearchMaxRings do
        local r = ring * P.goalSearchStep
        for k = 0, 15 do
            local a = k * math.pi / 8
            local candidate = {pos[1] + r * math.cos(a), pos[2] + r * math.sin(a), pos[3]}
            if not checkCollidesAt(candidate) then return candidate end
        end
    end
    return nil
end

----------------------------------------------------------------------
-- Goal acquisition (latency + smoothing)
----------------------------------------------------------------------

--- Goal position as the robot *perceives* it: delayed by `goalLatency`
-- and low-pass filtered so sensor jitter does not trigger re-planning.
local function getTargetPosition()
    local now = sim.getSimulationTime()
    goalBuffer[#goalBuffer + 1] = {t = now, p = sim.getObjectPosition(goalDummy, -1)}

    local delayed = goalBuffer[1].p
    while #goalBuffer > 1 and goalBuffer[1].t < now - P.goalLatency do
        delayed = goalBuffer[1].p
        table.remove(goalBuffer, 1)
    end

    if not trackedGoal then
        trackedGoal = {delayed[1], delayed[2], delayed[3]}
    else
        local a = P.goalSmoothing
        for i = 1, 3 do
            trackedGoal[i] = (1 - a) * trackedGoal[i] + a * delayed[i]
        end
    end
    return {trackedGoal[1], trackedGoal[2], trackedGoal[3]}
end

----------------------------------------------------------------------
-- Visualisation
----------------------------------------------------------------------
local function visualizePath(path)
    if not P.drawPath or not pathLineCont then return end
    sim.addDrawingObjectItem(pathLineCont, nil)   -- clear
    if not path then return end
    local z = sim.getObjectPosition(robot, -1)[3]
    for i = 1, #path // 2 - 1 do
        sim.addDrawingObjectItem(pathLineCont, {
            path[(i - 1) * 2 + 1], path[(i - 1) * 2 + 2], z,
            path[i * 2 + 1],       path[i * 2 + 2],       z,
        })
    end
end

--- Show the states the planner accepted as collision-free.
local function visualizeCollisionFreeNodes(states)
    if not nodeDrawCont then return end
    sim.addDrawingObjectItem(nodeDrawCont, nil)
    if not states then return end
    local z = sim.getObjectPosition(robot, -1)[3]
    for i = 1, #states // 2 do
        sim.addDrawingObjectItem(nodeDrawCont, {states[(i - 1) * 2 + 1], states[(i - 1) * 2 + 2], z})
    end
end

----------------------------------------------------------------------
-- Planning
----------------------------------------------------------------------

--- Plan a 2D collision-free path from `startPos` to `goalPos`.
-- Returns a flat {x1,y1, x2,y2, ...} table, or nil after all attempts fail.
local function planPath(startPos, goalPos)
    local task = simOMPL.createTask('hriMazeTask')
    local space = {
        simOMPL.createStateSpace('2d', simOMPL.StateSpaceType.position2d,
            collisionVolume,
            {startPos[1] - P.searchRange, startPos[2] - P.searchRange},
            {startPos[1] + P.searchRange, startPos[2] + P.searchRange}, 1),
    }
    simOMPL.setStateSpace(task, space)
    simOMPL.setAlgorithm(task, simOMPL.Algorithm[P.plannerName])
    simOMPL.setCollisionPairs(task, collisionPairs)
    simOMPL.setStartState(task, {startPos[1], startPos[2]})
    simOMPL.setGoalState(task,  {goalPos[1],  goalPos[2]})
    simOMPL.setup(task)

    local path = nil
    for attempt = 1, P.planningAttempts do
        local solved, candidate = simOMPL.compute(task, P.planningTime, -1, P.pathStateCount)
        if solved and candidate and #candidate >= 4 then
            path = candidate
            break
        end
        sim.addLog(sim.verbosity_scriptwarnings,
            ('planning attempt %d/%d failed'):format(attempt, P.planningAttempts))
    end

    simOMPL.destroyTask(task)
    return path
end

----------------------------------------------------------------------
-- Motion control
----------------------------------------------------------------------

--- Differential-drive speeds for a heading error, saturated symmetrically
-- so the robot slows into turns instead of stalling on one wheel.
local function motorSpeedsForHeading(headingError)
    local turn    = P.headingGain * headingError
    local forward = P.cruiseSpeed * math.max(0.15, math.cos(headingError))
    local left    = forward - turn
    local right   = forward + turn
    local peak    = math.max(math.abs(left), math.abs(right))
    if peak > P.maxWheelSpeed then
        local scale = P.maxWheelSpeed / peak
        left, right = left * scale, right * scale
    end
    return left, right
end

--- Signed shortest angular difference, wrapped to [-pi, pi].
local function wrapAngle(a)
    while a > math.pi do a = a - 2 * math.pi end
    while a < -math.pi do a = a + 2 * math.pi end
    return a
end

local function driveTowards(point)
    local pos = sim.getObjectPosition(robotRef, -1)
    local ori = sim.getObjectOrientation(robotRef, -1)
    local desired = math.atan2(point[2] - pos[2], point[1] - pos[1])
    local left, right = motorSpeedsForHeading(wrapAngle(desired - ori[3]))
    sim.setJointTargetVelocity(leftMotor, left)
    sim.setJointTargetVelocity(rightMotor, right)
    return math.sqrt((point[1] - pos[1])^2 + (point[2] - pos[2])^2)
end

local function stopMotors()
    sim.setJointTargetVelocity(leftMotor, 0)
    sim.setJointTargetVelocity(rightMotor, 0)
end

----------------------------------------------------------------------
-- Main thread
----------------------------------------------------------------------
function sysCall_thread()
    -- 1. Nudge the robot out of any start pose that already collides.
    local startPos = sim.getObjectPosition(robot, -1)
    local freeStart = findFreePositionNear(startPos)
    if freeStart and (freeStart[1] ~= startPos[1] or freeStart[2] ~= startPos[2]) then
        sim.setObjectPosition(robot, -1, freeStart)
        sim.addLog(sim.verbosity_scriptinfos, 'robot displaced to a collision-free start')
    end

    local path, waypointIndex, plannedGoal = nil, 1, nil

    while true do
        -- The per-iteration work lives in its own block so that `goto continue`
        -- never jumps into the scope of a local (a Lua 5.4 compile error).
        do
            local robotPos = sim.getObjectPosition(robotRef, -1)
            local goalPos  = getTargetPosition()

            -- 2. A goal buried inside an obstacle is relocated, not rejected.
            local reachableGoal = findFreePositionNear(goalPos)
            if not reachableGoal then
                stopMotors()
                sim.addLog(sim.verbosity_scriptwarnings, 'no collision-free goal in reach')
                goto continue
            end

            -- 3. Re-plan on the first pass, and whenever the goal has drifted.
            local goalMoved = plannedGoal and
                math.sqrt((reachableGoal[1] - plannedGoal[1])^2 +
                          (reachableGoal[2] - plannedGoal[2])^2) > P.goalMoveThreshold
            if not path or goalMoved then
                stopMotors()
                path = planPath(robotPos, reachableGoal)
                plannedGoal, waypointIndex = reachableGoal, 1
                visualizePath(path)
                visualizeCollisionFreeNodes(path)
                if not path then
                    sim.addLog(sim.verbosity_scripterrors, 'planner found no route to the goal')
                    goto continue
                end
            end

            -- 4. Follow the path, retiring waypoints as they are reached.
            local waypointCount = #path // 2
            if waypointIndex > waypointCount then
                if driveTowards(reachableGoal) < P.goalTolerance then
                    stopMotors()
                    sim.addLog(sim.verbosity_scriptinfos, 'goal reached')
                    break
                end
            else
                local wp = {path[(waypointIndex - 1) * 2 + 1], path[(waypointIndex - 1) * 2 + 2]}
                if driveTowards(wp) < P.waypointTolerance then
                    waypointIndex = waypointIndex + 1
                end
            end

            if P.drawTrace and traceLineCont then
                sim.addDrawingObjectItem(traceLineCont, robotPos)
            end

            ::continue::
        end
        sim.step()
    end

    stopMotors()
end
