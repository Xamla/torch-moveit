ros = require 'ros'
moveit = require 'moveit'

ros.init('check_self_collision')

robot_model_loader = moveit.RobotModelLoader("robot_description")
kinematic_model = robot_model_loader:getModel()

print(kinematic_model:printModelInfo())

planning_scene = moveit.PlanningScene(kinematic_model)

local collision = planning_scene:checkSelfCollision()
print(collision)

planning_scene:release()


ros.shutdown()
