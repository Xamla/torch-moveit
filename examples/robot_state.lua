ros = require 'ros'
moveit = require 'moveit'

ros.init()
local sp = ros.AsyncSpinner()
sp:start()

g = moveit.MoveGroup('manipulator')
g:setStartStateToCurrentState()
s = g:getCurrentState()
c = s:getVariableCount()
print('Variable count: ' .. c)
n = s:getVariableNames()
print(tostring(n))
p = s:getVariablePositions()
v = s:getVariableVelocities()
print(p)

jac = s:getJacobian(g:getName())

print(jac)

vel = jac * v

print("Cartesian velocity: " .. tostring(vel))
print("Joint velocity:" .. tostring(v))
