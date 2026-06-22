sim = require'sim'
simROS2 = require'simROS2'

goalObjects = {}
currentGoalIndex = 1
odomScriptHandle = nil
justReset = false

function sysCall_init()
    goalObjects[1] = sim.getObject('/Goal_Mesa1')
    goalObjects[2] = sim.getObject('/Goal_Mesa2')
    goalObjects[3] = sim.getObject('/Goal_Mesa3')

    local ok, handle = pcall(sim.getObject, '/odom/OdomScript')
    if ok then
        odomScriptHandle = handle
    end

    simROS2.createService('table_goals', 'std_srvs/srv/Trigger', 'tableGoalsCallback')
    simROS2.createService('/new_goal', 'std_srvs/srv/Trigger', 'newGoal_cb')
    simROS2.createService('/reset_simulation', 'std_srvs/srv/Empty', 'resetSim_cb')

    moveGoalToTable(1)
    sim.addLog(sim.verbosity_scriptinfos, 'Waiter goals script iniciado. Goal en Mesa 1')
end

function moveGoalToTable(index)
    local pos = sim.getObjectPosition(goalObjects[index], sim.handle_world)
    local goal = sim.getObject('/goal')
    sim.setObjectPosition(goal, pos, sim.handle_world)
    currentGoalIndex = index
end

function resetGoal()
    moveGoalToTable(1)
end

function resetSim_cb(req_srv)
    if odomScriptHandle then
        pcall(sim.callScriptFunction, 'resetRobot', odomScriptHandle)
    end
    resetGoal()
    justReset = true
    return {}
end

function tableGoalsCallback(req)
    local names = {'Goal_Mesa1', 'Goal_Mesa2', 'Goal_Mesa3'}
    local tables = {}

    for i = 1, #names do
        local ok, handle = pcall(sim.getObject, '/' .. names[i])
        if not ok or handle == -1 then
            return {
                success = false,
                message = 'No se encontro el dummy ' .. names[i]
            }
        end

        local position = sim.getObjectPosition(handle, sim.handle_world)
        tables[#tables + 1] = string.format(
            '{"name":"%s","x":%.6f,"y":%.6f}',
            names[i],
            position[1],
            position[2]
        )
    end

    return {
        success = true,
        message = '{"tables":[' .. table.concat(tables, ',') .. ']}'
    }
end

function newGoal_cb(srv_req)
    if not justReset then
        local nextIndex = currentGoalIndex + 1
        if nextIndex > #goalObjects then
            nextIndex = 1
        end
        moveGoalToTable(nextIndex)
    else
        justReset = false
    end

    local pos = sim.getObjectPosition(goalObjects[currentGoalIndex], sim.handle_world)
    local json = require("dkjson")
    local position = json.encode({x = pos[1], y = pos[2]})
    local resp = {
        success = true,
        message = position
    }
    return resp
end

function sysCall_actuation()
end

function sysCall_sensing()
end

function sysCall_cleanup()
end
