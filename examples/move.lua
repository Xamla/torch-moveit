ros = require 'ros'
moveit = require 'moveit'

ros.init('test')
local sp = ros.AsyncSpinner()
sp:start()

g = moveit.MoveGroup('arm')
pos, rot = g:getCurrentPose_Tensors()
print('pose:'..tostring(pos)..tostring(rot))
g:setPositionTarget(0.3, 0.5, 0.2)
s, p = g:plan()
g:execute(p)

