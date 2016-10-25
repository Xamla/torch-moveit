ros = require 'ros'
moveit = require 'moveit'

ros.init('robot_trajectory')
local sp = ros.AsyncSpinner()
sp:start()

robot_model_loader = moveit.RobotModelLoader("robot_description")
kinematic_model = robot_model_loader:getModel()

print(kinematic_model:printModelInfo())

traj = moveit.RobotTrajectory(kinematic_model, "manipulator")

g = moveit.MoveGroup('manipulator')
s = g:getCurrentState()
traj:addSuffixWayPoint(s,0.008)
traj:addSuffixWayPoint(s,0.008)
traj:addSuffixWayPoint(s,0.008)
traj:addSuffixWayPoint(s,0.008)
traj:addPrefixWayPoint(s,0.008)
print(traj:getGroupName())

print("Waypoint count: " .. traj:getWayPointCount())

print(traj:getLastWayPoint():getVariablePositions())
print(traj:getFirstWayPoint():getVariablePositions())
print(traj:getRobotTrajectoryMsg())
ros.shutdown()
