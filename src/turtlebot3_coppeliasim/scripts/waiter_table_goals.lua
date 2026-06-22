-- Child script for turtlebot3_burger_ROS2_dqn_waiter.ttt.
-- It exposes the real world positions of the dummy objects Goal_Mesa1..3.

sim = require('sim')
simROS2 = require('simROS2')

function sysCall_init()
    tableGoalService = simROS2.createService(
        'table_goals',
        'std_srvs/srv/Trigger',
        tableGoalsCallback
    )
end

function sysCall_cleanup()
    if tableGoalService then
        simROS2.shutdownService(tableGoalService)
    end
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
