moveit = require 'moveit'

ros.init()
local sp = ros.AsyncSpinner()
sp:start()

g = moveit.MoveGroup('arm')
s = g:getCurrentState()
c = s:getVariableCount()
print('Variable count: ' .. c)
n = s:getVariableNames()
print(tostring(n))
p = s:getVariablePositions()
print(p)
