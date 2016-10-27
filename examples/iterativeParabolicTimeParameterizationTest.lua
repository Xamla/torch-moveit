ros = require 'ros'
tf = ros.tf
moveit = require 'moveit'

ros.init('robot_trajectory')
local sp = ros.AsyncSpinner()
sp:start()

robot_model_loader = moveit.RobotModelLoader("robot_description")
traj = moveit.RobotTrajectory(robot_model_loader:getModel(), "manipulator")

g = moveit.MoveGroup('manipulator')
local position={}
local rotation ={}
local cp = g:getCurrentPose()
local pose = g:getCurrentPose_StampedTransform()
print('pose/pos/rot:'..tostring(pose:getOrigin())..tostring(pose:getRotation()))

--pose:setOrigin({0.1, 0.2, 0.1})
local end_pose = tf.Transform()
end_pose:setOrigin(pose:getOrigin()-0.1)
end_pose:setRotation(pose:getRotation())
print('end_pose:'..tostring(end_pose))

local stepsize = 0.01 
position[1] = pose:getOrigin()
rotation[1] = pose:getRotation()
position[2] = end_pose:getOrigin()
rotation[2] = end_pose:getRotation()
local s,p, errCode = g:computeCartesianPath_Tensor(position,rotation,stepsize,100.0,false)
print(errCode)
iptp = moveit.IterativeParabolicTimeParameterization()
print(s)
if s then
  print("p:getTrajectoryMsg()")
  print("traj:setRobotTrajectoryMsg")
print(torch.type(g:getCurrentState()))
  traj:setRobotTrajectoryMsg(g:getCurrentState(),p:getTrajectoryMsg())
  print("  iptp:computeTimeStamps(traj)")
  iptp:computeTimeStamps(traj)
  traj:unwind()
  print("traj:getRobotTrajectoryMsg()")
  local plan = moveit.Plan()
  local msg = g:getCurrentState():toRobotStateMsg()
  p:setStartStateMsg(msg)
  print("plan:setTrajectoryMsg()")
  plan:setTrajectoryMsg(traj:getRobotTrajectoryMsg())
  print("plan:plot()")
  plan:plot(1)
end

ros.shutdown()
