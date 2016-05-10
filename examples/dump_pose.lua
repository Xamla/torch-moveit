ros = require 'ros'
moveit = require 'moveit'

ros.init('dump_pose')
ros.Time.init()
local sp = ros.AsyncSpinner()
sp:start()
g = moveit.MoveGroup('manipulator')

while ros.ok() do
  local st = g:getCurrentPose_StampedTransform()

  local origin = st:getOrigin()
  local rpy = st:getRotation():getRPY()

  print('origin: ' .. string.format('{ x:%f, y:%f, z:%f, }', origin[1], origin[2], origin[3]) ..
    ' rotation: ' .. string.format('{ roll:%f, pitch:%f, yaw:%f }', rpy[1], rpy[2], rpy[3]))

  ros.Duration(1):sleep()
end

ros.shutdown()

