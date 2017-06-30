--- LUA wrapper for moveit planning environment
-- dependency to tourch.ros
-- @classmod MoveGroupInterface
local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local tf = ros.tf

local MoveGroupInterface = torch.class('moveit.MoveGroupInterface', moveit)

local f

function init()
  local MoveGroupInterface_method_names = {
    "new",
    "delete",
    "release",
    "getName",
    "getActiveJoints",
    "getJoints",
    "getJointNames",
    "getLinkNames",
    "getPlanningFrame",
    "getEndEffectorLink",
    "setGoalTolerance",
    "getGoalJointTolerance",
    "setGoalJointTolerance",
    "getGoalOrientationTolerance",
    "setGoalOrientationTolerance",
    "getGoalPositionTolerance",
    "setGoalPositionTolerance",
    "setMaxVelocityScalingFactor",
    "setPlannerId",
    "getPlannigTime",
    "setPlanningTime",
    "getVariableCount",
    "setNumPlanningAttempts",
    "setStartStateToCurrentState",
    "setSupportSurfaceName",
    "setWorkspace",
    "allowLooking",
    "allowReplanning",
    "setRandomTarget",
    "setNamedTarget",
    "setPositionTarget",
    "setPositionTarget_Tensor",
    "setOrientationTarget",
    "setOrientationTarget_Transform",
    "setRPYTarget",
    "setPoseTarget_Tensor",
    "setPoseTarget_Pose",
    "setPoseReferenceFrame",
    "setEndEffectorLink",
    "getEndEffectorlink",
    "setJointValueTarget",
    "getJointValueTarget",
    "clearPoseTarget",
    "clearPoseTargets",
    "asyncMove",
    "move",
    "plan",
    "asyncExecute",
    "execute",
    "setJointPostureConstraint",
    "setOrientationConstraint",
    "clearPathConstraints",
    "computeCartesianPath_Tensor",
    "attachObject",
    "detachObject",
    "stop",
    "startStateMonitor",
    "getCurrentState",
    "getCurrentPose_Tensor",
    "getCurrentPose_StampedTransform",
    "getCurrentPose",
    "pick"
  }

  f = utils.create_method_table("moveit_MoveGroupInterface_", MoveGroupInterface_method_names)
end

init()

--- Init function.
-- Specify the group name for which to construct this commander instance.
-- Throws an exception if there is an initialization error.
-- @tparam string name of the kinematic move group
function MoveGroupInterface:__init(name)
  print('test')
  self.o = f.new(name)
end

function MoveGroupInterface:release()
  f.release(self.o)
end

function MoveGroupInterface:cdata()
  return self.o
end

--- Get the name of the group this instance operates on.
-- @treturn string
function MoveGroupInterface:getName()
  return ffi.string(f.getName(self.o))
end

--- Get the name of the frame in which the robot is planning.
-- @treturn string
function MoveGroupInterface:getPlanningFrame()
  return ffi.string(f.getPlanningFrame(self.o))
end

--- Get the current end-effector link.
-- This returns the value set by setEndEffectorLink().
-- If setEndEffectorLink() was not called, this function reports the link name that serves as parent of an end-effector attached to this group.
-- If there are multiple end-effectors, one of them is returned. If no such link is known, the empty string is returned.
-- @treturn string
function MoveGroupInterface:getEndEffectorLink()
  return ffi.string(f.getEndEffectorLink(self.o))
end

--- Set the JointValueTarget and use it for future planning requests.
-- group_variable_values MUST contain exactly one value per joint variable in the same order as returned by getJointValueTarget().getJointModelGroup(getName())->getVariableNames().
-- This always sets all of the group's joint values.
-- After this call, the JointValueTarget is used instead of any previously set Position, Orientation, or Pose targets.
-- If these values are out of bounds then false is returned BUT THE VALUES ARE STILL SET AS THE GOAL.
-- @tparam torch.Tensor
-- @treturn bool
function MoveGroupInterface:setJointValueTarget(group_variable_values)
  return f.setJointValueTarget(self.o, group_variable_values:cdata())
end

--- Get the currently set joint state goal.
-- @tparam[opt] robot_state_output robot_state_output can be empty or can be used as target for the result
-- @treturn moveit.RobotState Joint value target as moveit.RobotState
function MoveGroupInterface:getJointValueTarget(robot_state_output)
  robot_state_output = robot_state_output or moveit.RobotState.createEmpty()
  f.getJointValueTarget(self.o, robot_state_output:cdata())
  return robot_state_output
end

--- Get only the active (actuated) joints this instance operates on.
-- @tparam[opt] output output can be empty or can be used as target for the result
-- @return moveit.Strings
function MoveGroupInterface:getActiveJoints(output)
  output = output or std.StringVector()
  f.getActiveJoints(self.o, output:cdata())
  return output
end

--- Get all the joints this instance operates on (including fixed joints).
-- @tparam[opt] output output can be empty or can be used as target for the result
-- @return moveit.Strings
function MoveGroupInterface:getJoints(output)
  output = output or std.StringVector()
  f.getJoints(self.o, output:cdata())
  return output
end


--- Get vector of names of joints available in move group.
-- @tparam[opt] output output can be empty or can be used as target for the result
-- @return moveit.Strings
function MoveGroupInterface:getJointNames(output)
  output = output or std.StringVector()
  f.getJointNames(self.o, output:cdata())
  return output
end


--- Get vector of names of links available in move group.
-- @tparam[opt] output output can be empty or can be used as target for the result
-- @return moveit.Strings
function MoveGroupInterface:getLinkNames(output)
  output = output or std.StringVector()
  f.getLinkNames(self.o, output:cdata())
  return output
end

--- Set the tolerance that is used for reaching the goal.
-- For joint state goals, this will be distance for each joint, in the configuration space (radians or meters depending on joint type).
-- For pose goals this will be the radius of a sphere where the end-effector must reach.
-- This function simply triggers calls to setGoalPositionTolerance, setGoalOrientationTolerance and setGoalJointTolerance.
-- @tparam number tolerance Goal error tolerance radians or meters depending on joint type
-- @see setGoalPositionTolerance
-- @see setGoalOrientationTolerance
-- @see setGoalJointTolerance
function MoveGroupInterface:setGoalTolerance(tolerance)
  f.setGoalTolerance(self.o, tolerance)
end

--- Get the tolerance that is used for reaching a joint goal.
-- Distance for each joint in configuration space.
-- @treturn number Goal error tolerance in rad
function MoveGroupInterface:getGoalJointTolerance()
  return f.getGoalJointTolerance(self.o)
end

--- Set the joint tolerance (for each joint) that is used for reaching the goal when moving to a joint value target.
-- @tparam number tolerance Goal error tolerance in rad
function MoveGroupInterface:setGoalJointTolerance(tolerance)
  f.setGoalJointTolerance(self.o, tolerance)
end

--- Get the tolerance that is used for reaching an orientation goal.
-- This is the tolerance for roll, pitch and yaw, in radians.
-- @return Goal tolerance in radians
function MoveGroupInterface:getGoalOrientationTolerance()
  return f.getGoalOrientationTolerance(self.o)
end

--- Set the orientation tolerance that is used for reaching the goal when moving to a pose.
-- @tparam number tolerance Goal tolerance in radians
function MoveGroupInterface:setGoalOrientationTolerance(tolerance)
  f.setGoalOrientationTolerance(self.o, tolerance)
end

--- Get the tolerance that is used for reaching a position goal.
-- This is the radius of a sphere where the end-effector must reach.
-- @treturn number Goal tolerance in meter
function MoveGroupInterface:getGoalPositionTolerance()
  return f.getGoalPositionTolerance(self.o)
end

--- Set the position tolerance that is used for reaching the goal when moving to a pose
-- @tparam number tolerance Goal tolerance in meter
function MoveGroupInterface:setGoalPositionTolerance(tolerance)
  f.setGoalPositionTolerance(self.o, tolerance)
end

--- Set a scaling factor for optionally reducing the maximum joint velocity.
-- Allowed values are in [0,1]. The maximum joint velocity specified in the robot model is multiplied by the factor.
-- If outside valid range (imporantly, this includes it being set to 0.0), the factor is set to a default value of 1.0 internally (i.e. maximum joint velocity).
-- @tparam int factor between [0,1]
function MoveGroupInterface:setMaxVelocityScalingFactor(factor)
  f.setMaxVelocityScalingFactor(self.o, factor)
end

--- Specify a planner to be used for further planning.
-- @tparam string name
function MoveGroupInterface:setPlannerId(name)
  f.setPlannerId(self.o, name)
end

--- Get the number of seconds set by setPlanningTime(seconds).
-- @see setPlanningTime
-- @treturn number
function MoveGroupInterface:getPlannigTime()
  return f.getPlannigTime(self.o)
end

--- Specify the maximum amount of time to use when planning.
-- @tparam number seconds
function MoveGroupInterface:setPlanningTime(seconds)
  f.setPlanningTime(self.o, seconds)
end

--- Get the number of variables used to describe the state of this group.
--  This is larger or equal to the number of DOF.
-- @return int
function MoveGroupInterface:getVariableCount()
  return f.getVariableCount(self.o)
end

--- Set the number of times the motion plan is to be computed from scratch before the shortest solution is returned.
-- The default value is 1.
-- @tparam int attempts
function MoveGroupInterface:setNumPlanningAttempts(attempts)
  f.setNumPlanningAttempts(self.o, attempts)
end

--- Set the starting state for planning to be that reported by the robot's joint state publication.
function MoveGroupInterface:setStartStateToCurrentState()
  f.setStartStateToCurrentState(self.o)
end

--- For pick/place operations, the name of the support surface is used to specify the fact that attached objects are allowed to touch the support surface.
-- @tparam string name
function MoveGroupInterface:setSupportSurfaceName(name)
  f.setSupportSurfaceName(self.o, name)
end

--- function setWorkspace
-- Specify the workspace bounding box. The box is specified in the planning frame (i.e. relative to the robot root link start position).
-- This is useful when the MoveGroupInterface's group contains the root joint of the robot -- i.e. when planning motion for the robot relative to the world.
-- @tparam number minx
-- @tparam number miny
-- @tparam number minz
-- @tparam number maxx
-- @tparam number maxy
-- @tparam number maxz
-- @tparam[opt] string end_effector_link should be the name of the use end effector link
function MoveGroupInterface:setWorkspace(minx, miny, minz, maxx, maxy, maxz, end_effector_link)
  f.setWorkspace(self.o, minx, miny, minz, maxx, maxy, maxz, end_effector_link or ffi.NULL)
end

--- Specify whether the robot is allowed to look around before moving if it determines it should.
-- (default is true).
-- @tparam[opt=true] boolean flag
function MoveGroupInterface:allowLooking(flag)
  f.allowLooking(self.o, flag or true)
end

--- Specify whether the robot is allowed to replan if it detects changes in the environment.
-- (default is true)
-- @tparam[opt=true] boolean flag
function MoveGroupInterface:allowReplanning(flag)
  f.allowReplanning(self.o, flag or true)
end

--- Set the joint state goal to a random joint configuration.
-- After this call, the JointValueTarget is used instead of any previously set Position, Orientation, or Pose targets.
function MoveGroupInterface:setRandomTarget()
  f.setRandomTarget(self.o)
end

--- Set the current joint values to be ones previously remembered by rememberJointValues() or,
-- if not found, that are specified in the SRDF under the name name as a group state.
-- @tparam string name
function MoveGroupInterface:setNamedTarget(name)
  f.setNamedTarget(self.o, name)
end

--- Set the goal position of the end-effector end_effector_link to be (x, y, z).
-- If end_effector_link is empty then getEndEffectorLink() is used.
-- This new position target replaces any pre-existing JointValueTarget or pre-existing Position, Orientation, or Pose target for this end_effector_link.
-- @tparam ?torch.DoubleTensor|double x
-- @tparam[opt] double y
-- @tparam[opt] double z
-- @tparam[opt] string end_effector_link
-- @see getEndEffectorLink
function MoveGroupInterface:setPositionTarget(x, y, z, end_effector_link)
  if torch.isTensor(x) then
    return f.setPositionTarget_Tensor(self.o, x:cdata(), y or end_effector_link or ffi.NULL)
  else
    return f.setPositionTarget(self.o, x, y, z, end_effector_link or ffi.NULL)
  end
end

--- Set the goal orientation of the end-effector end_effector_link to be the quaternion (x,y,z,w).
-- If end_effector_link is empty then getEndEffectorLink() is used.
-- This new orientation target replaces any pre-existing JointValueTarget or pre-existing Position, Orientation, or Pose target for this end_effector_link.
-- @tparam ?torch.DoubleTensor|double x
-- @tparam[opt] double y
-- @tparam[opt] double z
-- @tparam[opt] double w
-- @tparam[opt] string end_effector_link
function MoveGroupInterface:setOrientationTarget(x, y, z, w, end_effector_link)
  if torch.isTensor(x) then
    return f.setOrientationTarget_Tensor(self.o, x:cdata(), y or end_effector_link or ffi.NULL)
  else
    return f.setOrientationTarget(self.o, x, y, z, w, end_effector_link or ffi.NULL)
  end
end

--- Set the goal orientation of the end-effector end_effector_link to be (roll,pitch,yaw) radians.
-- If end_effector_link is empty then getEndEffectorLink() is used.
-- This new orientation target replaces any pre-existing JointValueTarget or pre-existing Position, Orientation, or Pose target for this end_effector_link.
-- @tparam double roll
-- @tparam double pitch
-- @tparam double yaw
-- @tparam[opt] string end_effector_link
-- @see getEndEffectorLink
function MoveGroupInterface:setRPYTarget(roll, pitch, yaw, end_effector_link)
  return f.setRPYTarget(self.o, roll, pitch, yaw, end_effector_link or ffi.NULL)
end

--- Set the goal pose of the end-effector end_effector_link.
-- If end_effector_link is empty then getEndEffectorLink() is used.
-- This new pose target replaces any pre-existing JointValueTarget or pre-existing Position, Orientation, or Pose target for this end_effector_link.
-- @tparam torch.DoubleTensor target
-- @tparam[opt] string end_effector_link
-- @see getEndEffectorLink
function MoveGroupInterface:setPoseTarget(target, end_effector_link)
  if torch.isTensor(target) then
    return f.setPoseTarget_Tensor(self.o, target:cdata(), end_effector_link or ffi.NULL)
  elseif torch.isTypeOf(target, tf.Transform) then
    return f.setPoseTarget_Pose(self.o, target:cdata(), end_effector_link or ffi.NULL)
  else
    error('Invalid target type specified. Expected torch.DoubleTensor or tf.Transform.')
  end
end

--- Specify which reference frame to assume for poses specified without a reference frame.
-- @tparam string frame_name
function MoveGroupInterface:setPoseReferenceFrame(frame_name)
  f.setPoseReferenceFrame(self.o, frame_name)
end

--- Specify the parent link of the end-effector.
-- This end_effector_link will be used in calls to pose target functions when end_effector_link is not explicitly specified.
-- @tparam string name
function MoveGroupInterface:setEndEffectorLink(name)
  f.setEndEffectorLink(self.o, name)
end

--- Get the current end-effector link.
-- This returns the value set by setEndEffectorLink().
-- If setEndEffectorLink() was not called, this function reports the link name that serves as parent of an end-effector attached to this group.
-- If there are multiple end-effectors, one of them is returned. If no such link is known, the empty string is returned.
-- @treturn string
-- @see setEndEffectorLink
function MoveGroupInterface:getEndEffectorlink()
  return ffi.string(f.getEndEffectorlink(self.o))
end

--- Forget pose(s) specified for end_effector_link.
function MoveGroupInterface:clearPoseTarget()
  f.clearPoseTarget(self.o)
end

--- Forget any poses specified for all end-effectors.
function MoveGroupInterface:clearPoseTargets()
  f.clearPoseTargets(self.o)
end

--- Plan and execute a trajectory that takes the group of joints declared in the constructor to the specified target.
-- This call is not blocking (does not wait for the execution of the trajectory to complete).
function MoveGroupInterface:moveAsync()
  return f.asyncMove(self.o)
end

--- Plan and execute a trajectory that takes the group of joints declared in the constructor to the specified target.
-- This call is always blocking (waits for the execution of the trajectory to complete) and requires an asynchronous spinner to be started.
function MoveGroupInterface:move()
  return f.move(self.o)
end

--- Compute a motion plan that takes the group declared in the constructor from the current state to the specified target.
-- No execution is performed. The resulting plan is stored in plan.
-- @tparam[opt] moveit.Plan plan_output
-- @treturn int status if the plan was generated successfully
-- @treturn moveit.Plan plan_output final plan
function MoveGroupInterface:plan(plan_output)
  if not plan_output then
    plan_output = moveit.Plan()
  end
  local status = f.plan(self.o, plan_output:cdata())
  return status, plan_output
end

--- Given a plan, execute it without waiting for completion.
-- Return true on success.
-- @tparam moveit.Plan plan
-- @treturn boolean success
function MoveGroupInterface:asyncExecute(plan)
  return f.asyncExecute(self.o, plan:cdata())
end

--- Given a plan, execute it while waiting for completion. Return true on success.
-- @tparam moveit.Plan plan
-- @treturn boolean success
function MoveGroupInterface:execute(plan)
  return f.execute(self.o, plan:cdata())
end

--- Specify a set of moveit_msgs::JointConstraint path constraints to use.
-- This replaces any path constraints set in previous calls to setPathConstraints().
-- @tparam string joint_name
-- @tparam int position
-- @tparam number tolerance_above
-- @tparam number tolerance_below
-- @tparam number weight
-- @see clearPathConstraints
function MoveGroupInterface:setJointPostureConstraint(joint_name, position, tolerance_above, tolerance_below , weight)
  f.setJointPostureConstraint(self.o,joint_name, position, tolerance_above, tolerance_below ,  weight)
end

--- Specify a set of moveit_msgs::OrientationConstraint path constraints to use.
-- This replaces any path constraints set in previous calls to setPathConstraints().
-- @tparam string link_name
-- @tparam string frame_id
-- @tparam number orientation_w
-- @tparam number absolute_x_axis_tolerance
-- @tparam number absolute_y_axis_tolerance
-- @tparam number absolute_z_axis_tolerance
-- @tparam number weight
-- @see clearPathConstraints
function MoveGroupInterface:setOrientationConstraint(link_name, frame_id, orientation_w, absolute_x_axis_tolerance, absolute_y_axis_tolerance, absolute_z_axis_tolerance, weight)
  f.setOrientationConstraint(self.o,link_name, frame_id, orientation_w, absolute_x_axis_tolerance, absolute_y_axis_tolerance,absolute_z_axis_tolerance, weight)
end

--- Specify that no path constraints are to be used.
-- This removes any path constraints set in previous calls to setPathConstraints().
function MoveGroupInterface:clearPathConstraints()
  f.clearPathConstraints(self.o)
end

--- Specify a set of moveit_msgs::OrientationConstraint path constraints to use.
-- This replaces any path constraints set in previous calls to setPathConstraints().
-- @tparam table positions This table should hold a number of waypoints 3D Tensors
-- @tparam table orientations This table should hold a number of orientations for each waypoint
-- @tparam number eef_step
-- @tparam number jump_threshold
-- @tparam[opt=true] boolean avoid_collisions
-- @treturn moveit.Plan
-- @see clearPathConstraints
function MoveGroupInterface:computeCartesianPath_Tensor(positions, orientations, eef_step, jump_threshold, avoid_collisions, plan_output)
  local avoid_collisions = avoid_collisions or true
  local plan_output = plan_output or moveit.Plan()
  if torch.type(positions) ~= 'table' then
    error('Argument positions of wrong type. Table expected.')
  end
  if torch.type(orientations) ~= 'table' then
    error('Argument orientations of wrong type. Table expected.')
  end

  local positions_ = torch.zeros(#positions, 3)         -- xyz
  local orientations_ = torch.zeros(#orientations, 4)
  for i=1, #positions do
    positions_[{i,{}}] = positions[i]
    local pose = orientations[i]
    local q = pose:toTensor()
    orientations_[{i,{}}] = q
  end

  local error_code = ffi.new 'int[1]'
  local status = f.computeCartesianPath_Tensor(self.o, positions_:cdata(), orientations_:cdata(), eef_step, jump_threshold, avoid_collisions, error_code, plan_output:cdata())
  return status, plan_output, error_code[0]
end

--- Given the name of an object in the planning scene, make the object attached to a link of the robot.
-- If no link name is specified, the end-effector is used. If there is no end-effector, the first link in the group is used.
-- If the object name does not exist an error will be produced in move_group, but the request made by this interface will succeed.
-- @tparam string object
-- @tparam[opt] string link
-- @treturn boolean
function MoveGroupInterface:attachObject(object, link)
  return f.attachObject(object, link or ffi.NULL)
end

--- Detach an object. name specifies the name of the object attached to this group, or the name of the link the object is attached to.
-- If there is no name specified, and there is only one attached object, that object is detached.
-- An error is produced if no object to detach is identified.
-- @tparam string object
-- @treturn boolean
function MoveGroupInterface:detachObject(object)
  return f.detachObject(object)
end

--- Stop any trajectory execution, if one is active.
function MoveGroupInterface:stop()
  f.stop(self.o)
end

--- When reasoning about the current state of a robot, a CurrentStateMonitor instance is automatically constructed.
-- This function allows triggering the construction of that object from the beginning, so that future calls to functions such as getCurrentState() will not take so long and are less likely to fail.
-- @tparam[opt=0.01] number wait
-- @treturn boolean
-- @see getCurrentState
function MoveGroupInterface:startStateMonitor(wait)
  return f.startStateMonitor(self.o, wait or 0.01)
end

--- Get the current state of the robot.
-- @treturn moveit.RobotState
function MoveGroupInterface:getCurrentState()
  return moveit.RobotState(f.getCurrentState(self.o))
end

--- Get the current state of the robot.
-- @tparam string end_effector_link
-- @tparam[opt] torch.DoubleTensor output end_effector_link
-- @treturn torch.DoubleTensor output
function MoveGroupInterface:getCurrentPose_Tensor(end_effector_link, output)
  output = output or torch.DoubleTensor()
  f.getCurrentPose_Transform(self.o, end_effector_link or ffi.NULL, output:cdata())
  return output
end

--- Get the current state of the robot.
-- @tparam string end_effector_link
-- @tparam[opt] tf.StampedTransform output
-- @treturn tf.StampedTransform
function MoveGroupInterface:getCurrentPose_StampedTransform(end_effector_link, output)
  output = output or tf.StampedTransform()
  f.getCurrentPose_StampedTransform(self.o, end_effector_link or ffi.NULL, output:cdata())
  return output
end

--- Get the current state of the robot.
-- @tparam string end_effector_link
-- @tparam[opt] tf.Transform output
-- @treturn tf.Transform
function MoveGroupInterface:getCurrentPose(end_effector_link, output)
  output = output or tf.Transform()
  f.getCurrentPose(self.o, end_effector_link or ffi.NULL, output:cdata())
  return output
end

--- Pick up an object.
-- @tparam string object Name of the object
-- @treturn MoveItErrorCode
function MoveGroupInterface:pick(object)
  --object needs to be string
  print(object)
  return f.pick(self.o,object)
end
