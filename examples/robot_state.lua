moveit = require 'moveit'
moveit.init()

g = moveit.MoveGroup('arm')
s = g:getCurrentState()
c = s:getVariableCount()
print('Variable count: ' .. c)
n = s:getVariableNames()
print(tostring(n))
p = s:getVariablePositions()
print(p)
