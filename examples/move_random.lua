moveit = require 'moveit'
ros = moveit.ros
tf = moveit.tf

ros.init('move_random')
ros.Time.init()

local b = tf.TransformBroadcaster()

local ps = moveit.PlanningSceneInterface()

g = moveit.MoveGroup('arm')

local pose = g:getCurrentPose_StampedTransform()
print('pose:'..tostring(pose:getOrigin())..tostring(pose:getRotation():getRPY()))
g:setMaxVelocityScalingFactor(0.1)

local sp = ros.AsyncSpinner()
sp:start()

--[[local c = moveit.CollisionObject()
c:addPlane({0, 0, 1, -0.001})
c:setId('gp1')
ps:addCollisionObject(c)]]
--ps:addPlane('ground plane', 0, 0, 1, 0.101)
--print('ground plane added')

box_pose = tf.Transform()
box_pose:setOrigin({0.3, 0.3, 0})
ps:addBox('b1', 0.25, 0.25, 0.25, box_pose)
print('box added')

box_pose = tf.Transform()
box_pose:setOrigin({0.0, 0.0, -0.01})
ps:addBox('b2', 5.25, 5.25, 0.01, box_pose)

while ros.ok() do
  g:setStartStateToCurrentState()
  g:setRandomTarget()
  
  s, p = g:plan()
  if s then
    print('plan found!')
    g:execute(p)
  end
  
  ros.Duration(0.1):sleep()
end
