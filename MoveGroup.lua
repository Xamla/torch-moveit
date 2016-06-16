--- LUA wrapper for moveit planning environment
-- dependency to tourch.ros
-- @classmod moveit.MoveGroup
local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local tf = ros.tf

local MoveGroup = torch.class('moveit.MoveGroup', moveit)

local f

function init()
  local MoveGroup_method_names = {
    "new",
    "delete",
    "release",
    "getName",
    "getJoints",
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

  f = utils.create_method_table("moveit_MoveGroup_", MoveGroup_method_names)
end

init()

--- function init
-- Specify the group name for which to construct this commander instance.
-- Throws an exception if there is an initialization error.
-- @tparam string name of the kinematic move group
function MoveGroup:__init(name)
  self.o = f.new(name)
end

function MoveGroup:release()
  f.release(self.o)
end

function MoveGroup:cdata()
  return self.o
end

--- function getName
-- Get the name of the group this instance operates on.
-- @treturn string
function MoveGroup:getName()
  return ffi.string(f.getName(self.o))
end

--- function getPlanningFrame
-- Get the name of the frame in which the robot is planning.
-- @treturn string
function MoveGroup:getPlanningFrame()
  return ffi.string(f.getPlanningFrame(self.o))
end

--- function getEndEffectorLink
-- Get the current end-effector link.
-- This returns the value set by setEndEffectorLink()
-- (or indirectly by setEndEffector()).
-- If setEndEffectorLink() was not called, this function reports the link name that serves as parent of an end-effector attached to this group.
-- If there are multiple end-effectors, one of them is returned. If no such link is known, the empty string is returned.
-- @treturn string
function MoveGroup:getEndEffectorLink()
  return ffi.string(f.getEndEffectorLink(self.o))
end

--- function getJoints
-- Get all the joints this instance operates on (including fixed joints)
-- @tparam[opt] strings strings be empty
-- @return moveit.Strings
function MoveGroup:getJoints(strings)
  strings = strings or moveit.Strings()
  f.getJoints(self.o, strings:cdata())
  return strings
end

--- function setGoalTolerance
-- Set the tolerance that is used for reaching the goal.
-- For joint state goals, this will be distance for each joint, in the configuration space (radians or meters depending on joint type).
-- For pose goals this will be the radius of a sphere where the end-effector must reach.
-- This function simply triggers calls to @see setGoalPositionTolerance(), @see setGoalOrientationTolerance() and @see setGoalJointTolerance().
-- @tparam number tolerance Goal error tolerance radians or meters depending on joint type
function MoveGroup:setGoalTolerance(tolerance)
  f.setGoalTolerance(self.o, tolerance)
end

--- function getGoalJointTolerance
-- Get the tolerance that is used for reaching a joint goal. This is distance for each joint in configuration space.
-- @treturn number Goal error tolerance in rad
function MoveGroup:getGoalJointTolerance()
  return f.getGoalJointTolerance(self.o)
end

--- function setGoalJointTolerance
-- Set the joint tolerance (for each joint) that is used for reaching the goal when moving to a joint value target.
-- @tparam number tolerance Goal error tolerance in rad
function MoveGroup:setGoalJointTolerance(tolerance)
  f.setGoalJointTolerance(self.o, tolerance)
end

--- function getGoalOrientationTolerance
-- Get the tolerance that is used for reaching an orientation goal. This is the tolerance for roll, pitch and yaw, in radians.
-- @return Goal tolerance in radians
function MoveGroup:getGoalOrientationTolerance()
  return f.getGoalOrientationTolerance(self.o)
end

--- function setGoalOrientationTolerance
-- Set the orientation tolerance that is used for reaching the goal when moving to a pose.
-- @tparam number tolerance Goal tolerance in radians
function MoveGroup:setGoalOrientationTolerance(tolerance)
  f.setGoalOrientationTolerance(self.o, tolerance)
end

--- function getGoalPositionTolerance
-- Get the tolerance that is used for reaching a position goal. This is be the radius of a sphere where the end-effector must reach.
-- @treturn number Goal tolerance in meter
function MoveGroup:getGoalPositionTolerance()
  return f.getGoalPositionTolerance(self.o)
end

--- function setGoalPositionTolerance
-- Set the position tolerance that is used for reaching the goal when moving to a pose.
-- @tparam Goal tolerance in meter
function MoveGroup:setGoalPositionTolerance(tolerance)
  f.setGoalPositionTolerance(self.o, tolerance)
end

--- function setMaxVelocityScalingFactor
-- Set a scaling factor for optionally reducing the maximum joint velocity.
-- Allowed values are in [0,1]. The maximum joint velocity specified in the robot model is multiplied by the factor.
-- If outside valid range (imporantly, this includes it being set to 0.0), the factor is set to a default value of 1.0 internally (i.e. maximum joint velocity)
-- @tparam scaling factor between [0,1]
function MoveGroup:setMaxVelocityScalingFactor(factor)
  f.setMaxVelocityScalingFactor(self.o, factor)
end

--- function setPlannerId
-- Specify a planner to be used for further planning.
-- @tparam string name
function MoveGroup:setPlannerId(name)
  f.setPlannerId(self.o, name)
end

--- function getPlannigTime
--  Get the number of seconds set by @see setPlanningTime(seconds)
-- @treturn number
function MoveGroup:getPlannigTime()
  return f.getPlannigTime(self.o)
end

--- function setPlanningTime
--  Specify the maximum amount of time to use when planning.
-- @tparam number seconds
function MoveGroup:setPlanningTime(seconds)
  f.setPlanningTime(self.o, seconds)
end

--- function getVariableCount
--  Get the number of variables used to describe the state of this group.
--  This is larger or equal to the number of DOF.
-- @return int
function MoveGroup:getVariableCount()
  return f.getVariableCount(self.o)
end

--- function setNumPlanningAttempts
-- Set the number of times the motion plan is to be computed from scratch before the shortest solution is returned.
-- The default value is 1.
-- @tparam int attempts
function MoveGroup:setNumPlanningAttempts(attempts)
  f.setNumPlanningAttempts(self.o, attempts)
end

--- function setStartStateToCurrentState
-- Set the starting state for planning to be that reported by the robot's joint state publication.
function MoveGroup:setStartStateToCurrentState()
  f.setStartStateToCurrentState(self.o)
end

--- function setSupportSurfaceName
-- For pick/place operations, the name of the support surface is used to specify the fact that attached objects are allowed to touch the support surface.
-- @tparam string name
function MoveGroup:setSupportSurfaceName(name)
  f.setSupportSurfaceName(self.o, name)
end

--- function setWorkspace
-- Specify the workspace bounding box. The box is specified in the planning frame (i.e. relative to the robot root link start position).
-- This is useful when the MoveGroup's group contains the root joint of the robot -- i.e. when planning motion for the robot relative to the world.
-- @tparam number minx
-- @tparam number miny
-- @tparam number minz
-- @tparam number maxx
-- @tparam number maxy
-- @tparam number maxz
-- @tparam[opt] string end_effector_link should be the name of the use end effector link
function MoveGroup:setWorkspace(minx, miny, minz, maxx, maxy, maxz, end_effector_link)
  f.setWorkspace(self.o, minx, miny, minz, maxx, maxy, maxz, end_effector_link or ffi.NULL)
end

--- function setSupportSurfaceName
-- Specify whether the robot is allowed to look around before moving if it determines it should (default is true)
-- @tparam[opt=true] bool flag
function MoveGroup:allowLooking(flag)
  f.allowLooking(self.o, flag or true)
end

--- function setSupportSurfaceName
-- Specify whether the robot is allowed to replan if it detects changes in the environment. (default is true)
-- @tparam[opt=true] bool flag
function MoveGroup:allowReplanning(flag)
  f.allowReplanning(self.o, flag or true)
end

--- function setSupportSurfaceName
-- Set the joint state goal to a random joint configuration.
-- After this call, the JointValueTarget is used instead of any previously set Position, Orientation, or Pose targets.
function MoveGroup:setRandomTarget()
  f.setRandomTarget(self.o)
end

--- function setNamedTarget
-- Set the current joint values to be ones previously remembered by @see rememberJointValues() or,
-- if not found, that are specified in the SRDF under the name name as a group state.
-- @tparam string name
function MoveGroup:setNamedTarget(name)
  f.setNamedTarget(self.o, name)
end

--- function setPositionTarget
-- Set the goal position of the end-effector end_effector_link to be (x, y, z).
-- @tparam ?Tensor|double x
-- @tparam[opt] double y
-- @tparam[opt] double z
-- @tparam[opt] string end_effector_link
function MoveGroup:setPositionTarget(x, y, z, end_effector_link)
  if torch.isTensor(x) then
    return f.setPositionTarget_Tensor(self.o, x:cdata(), y or end_effector_link or ffi.NULL)
  else
    return f.setPositionTarget(self.o, x, y, z, end_effector_link or ffi.NULL)
  end
end

--- function setOrientationTarget
-- Set the goal orientation of the end-effector end_effector_link to be the quaternion (x,y,z,w).
-- @tparam ?Tensor|double x
-- @tparam[opt] double y
-- @tparam[opt] double z
-- @tparam[opt] double w
-- @tparam[opt] string end_effector_link
function MoveGroup:setOrientationTarget(x, y, z, w, end_effector_link)
  if torch.isTensor(x) then
    return f.setOrientationTarget_Tensor(self.o, x:cdata(), y or end_effector_link or ffi.NULL)
  else
    return f.setOrientationTarget(self.o, x, y, z, w, end_effector_link or ffi.NULL)
  end
end

--- function setRPYTarget
-- Set the goal orientation of the end-effector end_effector_link to be (roll,pitch,yaw) radians.
-- If end_effector_link is empty then @see getEndEffectorLink() is used.
-- This new orientation target replaces any pre-existing JointValueTarget or pre-existing Position, Orientation, or Pose target for this end_effector_link.
-- @tparam double roll
-- @tparam double pitch
-- @tparam double yaw
-- @tparam[opt] string end_effector_link
function MoveGroup:setRPYTarget(roll, pitch, yaw, end_effector_link)
  return f.setRPYTarget(self.o, roll, pitch, yaw, end_effector_link or ffi.NULL)
end

--- function setPoseTarget
-- Set the goal pose of the end-effector end_effector_link.
-- If end_effector_link is empty then getEndEffectorLink() is used.
-- This new pose target replaces any pre-existing JointValueTarget or pre-existing Position, Orientation, or Pose target for this end_effector_link.
-- @tparam Tensor target
-- @tparam[opt] string end_effector_link
function MoveGroup:setPoseTarget(target, end_effector_link)
  if torch.isTensor(target) then
    return f.setPoseTarget_Tensor(self.o, target:cdata(), end_effector_link or ffi.NULL)
  end
  --f.setPoseTarget_Pose(self.o)
end

--- function setPoseReferenceFrame
--  Specify which reference frame to assume for poses specified without a reference frame.
-- @tparam string frame_name
function MoveGroup:setPoseReferenceFrame(frame_name)
  f.setPoseReferenceFrame(self.o, frame_name)
end

--- function setEndEffectorLink
-- Specify the parent link of the end-effector. This end_effector_link will be used in calls to pose target functions when end_effector_link is not explicitly specified.
-- @tparam string name
function MoveGroup:setEndEffectorLink(name)
  f.setEndEffectorLink(self.o, name)
end

--- function getEndEffectorlink
-- Get the current end-effector link.
-- This returns the value set by @see setEndEffectorLink() (or indirectly by @see setEndEffector()).
-- If @see setEndEffectorLink() was not called, this function reports the link name that serves as parent of an end-effector attached to this group.
-- If there are multiple end-effectors, one of them is returned. If no such link is known, the empty string is returned.
-- @treturn string
function MoveGroup:getEndEffectorlink()
  return ffi.string(f.getEndEffectorlink(self.o))
end

--- function clearPoseTarget
-- Forget pose(s) specified for end_effector_link.
function MoveGroup:clearPoseTarget()
  f.clearPoseTarget(self.o)
end

--- function clearPoseTargets
-- Forget any poses specified for all end-effectors.
function MoveGroup:clearPoseTargets()
  f.clearPoseTargets(self.o)
end

--- function moveAsync
-- Plan and execute a trajectory that takes the group of joints declared in the constructor to the specified target.
-- This call is not blocking (does not wait for the execution of the trajectory to complete).
function MoveGroup:moveAsync()
  return f.asyncMove(self.o)
end

--- function move
-- Plan and execute a trajectory that takes the group of joints declared in the constructor to the specified target.
-- This call is always blocking (waits for the execution of the trajectory to complete) and requires an asynchronous spinner to be started.
function MoveGroup:move()
  return f.move(self.o)
end

--- function plan
-- Compute a motion plan that takes the group declared in the constructor from the current state to the specified target.
-- No execution is performed. The resulting plan is stored in plan.
-- @tparam[opt] moveit.Plan() plan_output
-- @treturn int status if the plan was generated successfully
-- @treturn moveit.Plan() plan_output final plan
function MoveGroup:plan(plan_output)
  if not plan_output then
    plan_output = moveit.Plan()
  end
  local status = f.plan(self.o, plan_output:cdata())
  return status, plan_output
end

--- function asyncExecute
-- Given a plan, execute it without waiting for completion. Return true on success.
-- @tparam moveit.Plan() plan
-- @treturn bool success
function MoveGroup:asyncExecute(plan)
  return f.asyncExecute(self.o, plan:cdata())
end

--- function asyncExecute
-- Given a plan, execute it while waiting for completion. Return true on success.
-- @tparam moveit.Plan() plan
-- @treturn bool success
function MoveGroup:execute(plan)
  return f.execute(self.o, plan:cdata())
end

--- function setJointPostureConstraint
-- Specify a set of moveit_msgs::JointConstraint path constraints to use.
-- This replaces any path constraints set in previous calls to setPathConstraints().
-- @tparam string joint_name
-- @tparam int position
-- @tparam number tolerance_above
-- @tparam number tolerance_below
-- @tparam number weight
function MoveGroup:setJointPostureConstraint(joint_name, position, tolerance_above, tolerance_below , weight)
  f.setJointPostureConstraint(self.o,joint_name, position, tolerance_above, tolerance_below ,  weight)
end

--- function setOrientationConstraint
-- Specify a set of moveit_msgs::OrientationConstraint path constraints to use.
-- This replaces any path constraints set in previous calls to setPathConstraints().
-- @tparam string link_name
-- @tparam string frame_id
-- @tparam number orientation_w
-- @tparam number absolute_x_axis_tolerance
-- @tparam number absolute_y_axis_tolerance
-- @tparam number absolute_z_axis_tolerance
-- @tparam number weight
function MoveGroup:setOrientationConstraint(link_name, frame_id, orientation_w, absolute_x_axis_tolerance, absolute_y_axis_tolerance, absolute_z_axis_tolerance, weight)
  f.setOrientationConstraint(self.o,link_name, frame_id, orientation_w, absolute_x_axis_tolerance, absolute_y_axis_tolerance,absolute_z_axis_tolerance, weight)
end

--- function clearPathConstraints
--  Specify that no path constraints are to be used. This removes any path constraints set in previous calls to setPathConstraints().
function MoveGroup:clearPathConstraints()
  f.clearPathConstraints(self.o)
end

--- function computeCartesianPath_Tensor
-- Specify a set of moveit_msgs::OrientationConstraint path constraints to use.
-- This replaces any path constraints set in previous calls to setPathConstraints().
-- @tparam table positions This table should hold a number of waypoints 3D Tensors
-- @tparam table orientations This table should hold a number of orientations for each waypoint
-- @tparam number eef_step
-- @tparam number jump_threshold
-- @tparam[opt=true] bool avoid_collisions
-- @treturn moveit.Plan()
function MoveGroup:computeCartesianPath_Tensor(positions, orientations, eef_step, jump_threshold, avoid_collisions)
  local avoid_collisions = avoid_collisions or true
  if torch.type(positions) ~= 'table' then
    error('Argument positions of wrong type. Table expected.')
  end
  if torch.type(orientations) ~= 'table' then
    error('Argument orientations of wrong type. Table expected.')
  end
  if plan_output == nil then
    plan_output = moveit.Plan()
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
  return status, plan_output, error_code
end

--- function attachObject
-- Given the name of an object in the planning scene, make the object attached to a link of the robot.
-- If no link name is specified, the end-effector is used. If there is no end-effector, the first link in the group is used.
-- If the object name does not exist an error will be produced in move_group, but the request made by this interface will succeed.
-- @tparam string object
-- @tparam[opt] string link
-- @treturn bool
function MoveGroup:attachObject(object, link)
  return f.attachObject(object, link or ffi.NULL)
end

--- function detachObject
-- Detach an object. name specifies the name of the object attached to this group, or the name of the link the object is attached to.
-- If there is no name specified, and there is only one attached object, that object is detached.
-- An error is produced if no object to detach is identified.
-- @tparam string object
-- @treturn bool
function MoveGroup:detachObject(object)
  return f.detachObject(object)
end

--- function stop
-- Stop any trajectory execution, if one is active.
function MoveGroup:stop()
  f.stop(self.o)
end

--- function startStateMonitor
-- When reasoning about the current state of a robot, a CurrentStateMonitor instance is automatically constructed.
-- This function allows triggering the construction of that object from the beginning, so that future calls to functions such as @see getCurrentState() will not take so long and are less likely to fail.
-- @tparam[opt=0.01] number wait
-- @treturn bool
function MoveGroup:startStateMonitor(wait)
  return f.startStateMonitor(self.o, wait or 0.01)
end

--- function getCurrentState
-- Get the current state of the robot.
-- @treturn moveit.RobotState()
function MoveGroup:getCurrentState()
  return moveit.RobotState(f.getCurrentState(self.o))
end

--- function getCurrentPose_Tensor
-- Get the current state of the robot.
-- @tparam string end_effector_link
-- @tparam[opt] torch.DoubleTensor() output end_effector_link
-- @treturn torch.DoubleTensor() output
function MoveGroup:getCurrentPose_Tensor(end_effector_link, output)
  output = output or torch.DoubleTensor()
  f.getCurrentPose_Transform(self.o, end_effector_link or ffi.NULL, output:cdata())
  return output
end

--- function getCurrentPose_StampedTransform
-- Get the current state of the robot.
-- @tparam string end_effector_link
-- @tparam[opt] tf.StampedTransform() output
-- @treturn tf.StampedTransform()
function MoveGroup:getCurrentPose_StampedTransform(end_effector_link, output)
  output = output or tf.StampedTransform()
  f.getCurrentPose_StampedTransform(self.o, end_effector_link or ffi.NULL, output:cdata())
  return output
end

--- function getCurrentPose
-- Get the current state of the robot.
-- @tparam string end_effector_link
-- @tparam[opt] tf.Transform() output
-- @treturn tf.Transform()
function MoveGroup:getCurrentPose(end_effector_link, output)
  output = output or tf.Transform()
  f.getCurrentPose(self.o, end_effector_link or ffi.NULL, output:cdata())
  return output
end

--- function getCurrentPose
-- Pick up an object.
-- @tparam string object Name of the object
-- @treturn MoveItErrorCode
function MoveGroup:pick(object)
  --object needs to be string
  print(object)
  return f.pick(self.o,object)
end
