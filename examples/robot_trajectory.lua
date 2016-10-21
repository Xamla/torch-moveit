ros = require 'ros'
moveit = require 'moveit'

ros.init('robot_trajectory')

robot_model_loader = moveit.RobotModelLoader("robot_description")
kinematic_model = robot_model_loader:getModel()

print(kinematic_model:printModelInfo())

traj = moveit.RobotTrajectory(kinematic_model, "manipulator")

print(traj:getGroupName())

ros.shutdown()
