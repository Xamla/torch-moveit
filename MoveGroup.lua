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

function MoveGroup:__init(name)
  self.o = f.new(name)
end

function MoveGroup:release()
  f.release(self.o)
end

function MoveGroup:cdata()
  return self.o
end

function MoveGroup:getName()
  return ffi.string(f.getName(self.o))
end

function MoveGroup:getPlanningFrame()
  return ffi.string(f.getPlanningFrame(self.o))
end

function MoveGroup:getEndEffectorLink()
  return ffi.string(f.getEndEffectorLink(self.o))
end

function MoveGroup:getJoints(strings)
  strings = strings or moveit.Strings()
  f.getJoints(self.o, strings:cdata())
  return strings
end

function MoveGroup:setGoalTolerance(tolerance)
  f.setGoalTolerance(self.o, tolerance)
end

function MoveGroup:getGoalJointTolerance()
  return f.getGoalJointTolerance(self.o)
end

function MoveGroup:setGoalJointTolerance(tolerance)
  f.setGoalJointTolerance(self.o, tolerance)
end

function MoveGroup:getGoalOrientationTolerance()
  return f.getGoalOrientationTolerance(self.o)
end

function MoveGroup:setGoalOrientationTolerance(tolerance)
  f.setGoalOrientationTolerance(self.o, tolerance)
end

function MoveGroup:getGoalPositionTolerance()
  return f.getGoalPositionTolerance(self.o)
end

function MoveGroup:setGoalPositionTolerance(tolerance)
  f.setGoalPositionTolerance(self.o, tolerance)
end

function MoveGroup:setMaxVelocityScalingFactor(factor)
  f.setMaxVelocityScalingFactor(self.o, factor)
end

function MoveGroup:setPlannerId(name)
  f.setPlannerId(self.o, name)
end

function MoveGroup:getPlannigTime()
  return f.getPlannigTime(self.o)
end

function MoveGroup:setPlanningTime(seconds)
  f.setPlanningTime(self.o, seconds)
end

function MoveGroup:getVariableCount()
  return f.getVariableCount(self.o)
end

function MoveGroup:setNumPlanningAttempts(attempts)
  f.setNumPlanningAttempts(self.o, attempts)
end

function MoveGroup:setStartStateToCurrentState()
  f.setStartStateToCurrentState(self.o)
end

function MoveGroup:setSupportSurfaceName(name)
  f.setSupportSurfaceName(self.o, name)
end

function MoveGroup:setWorkspace(minx, miny, minz, maxx, maxy, maxz, end_effector_link)
  f.setWorkspace(self.o, minx, miny, minz, maxx, maxy, maxz, end_effector_link or ffi.NULL)
end

function MoveGroup:allowLooking(flag)
  f.allowLooking(self.o, flag)
end

function MoveGroup:allowReplanning(flag)
  f.allowReplanning(self.o, flag)
end

function MoveGroup:setRandomTarget()
  f.setRandomTarget(self.o)
end

function MoveGroup:setNamedTarget(name)
  f.setNamedTarget(self.o, name)
end

function MoveGroup:setPositionTarget(x, y, z, end_effector_link)
  if torch.isTensor(x) then
    return f.setPositionTarget_Tensor(self.o, x:cdata(), y or end_effector_link or ffi.NULL)
  else
    return f.setPositionTarget(self.o, x, y, z, end_effector_link or ffi.NULL)
  end
end

function MoveGroup:setOrientationTarget(x, y, z, w, end_effector_link)
  if torch.isTensor(x) then
    return f.setOrientationTarget_Tensor(self.o, x:cdata(), y or end_effector_link or ffi.NULL)
  else
    return f.setOrientationTarget(self.o, x, y, z, w, end_effector_link or ffi.NULL)
  end
end

function MoveGroup:setRPYTarget(roll, pitch, yaw, end_effector_link)
  return f.setRPYTarget(self.o, roll, pitch, yaw, end_effector_link or ffi.NULL)
end

function MoveGroup:setPoseTarget(target, end_effector_link)
  if torch.isTensor(target) then
    return f.setPoseTarget_Tensor(self.o, target:cdata(), end_effector_link or ffi.NULL)
  end
  --f.setPoseTarget_Pose(self.o)
end

function MoveGroup:setPoseReferenceFrame(frame_name)
  f.setPoseReferenceFrame(self.o, frame_name)
end

function MoveGroup:setEndEffectorLink(name)
  f.setEndEffectorLink(self.o, name)
end

function MoveGroup:getEndEffectorlink()
  return ffi.string(f.getEndEffectorlink(self.o))
end

function MoveGroup:clearPoseTarget()
  f.clearPoseTarget(self.o)
end

function MoveGroup:clearPoseTargets()
  f.clearPoseTargets(self.o)
end

function MoveGroup:moveAsync()
  return f.moveAsync(self.o)
end

function MoveGroup:move()
  return f.move(self.o)
end

function MoveGroup:plan(plan_output)
  if not plan_output then
    plan_output = moveit.Plan()
  end
  local status = f.plan(self.o, plan_output:cdata())
  return status, plan_output
end

function MoveGroup:asyncExecute(plan)
  return f.asyncExecute(self.o, plan:cdata())
end

function MoveGroup:execute(plan)
  return f.execute(self.o, plan:cdata())
end

function MoveGroup:setOrientationConstraint(link_name, frame_id, orientation_w, absolute_x_axis_tolerance, absolute_y_axis_tolerance, absolute_z_axis_tolerance, weight)
	f.setOrientationConstraint(self.o,link_name, frame_id, orientation_w, absolute_x_axis_tolerance, absolute_y_axis_tolerance,absolute_z_axis_tolerance, weight)
end

function MoveGroup:clearPathConstraints()
	f.clearPathConstraints(self.o)
end

function MoveGroup:computeCartesianPath_Tensor(positions, orientations, eef_step, jump_threshold, avoid_collisions)
  if not plan_output then
    plan_output = moveit.Plan()
  end
  print("MoveGroup:computeCartesianPath_Tensor")
       local error_code = ffi.new 'int[1]'
  if torch.type(positions) == 'table' and torch.type(orientations) == 'table' then
	tmp_positions = torch.zeros(#positions,3) -- needs to be xyz
	tmp_orientations = torch.zeros(#orientations,4) --computeCartesianPath expects orientations
	for i=1, #positions do
		tmp_positions[{i,{}}]=positions[i][{}]
		local pose = orientations[i]
--		local q = pose:getRotation():toTensor()
		local q = pose:toTensor()
                 tmp_orientations[{i,{}}]=q
	end
	local r = f.computeCartesianPath_Tensor(self.o, tmp_positions:cdata(), tmp_orientations:cdata(), eef_step, jump_threshold, avoid_collisions, error_code,plan_output:cdata())	
	if r > 0.5 then
		status = 1
		print("Cartesian Path successfully generated")
	else
		status = 0
		print("Cartesian Path NOT successfully generated")
	end
  else
print("WRONG DATA TYPE")
  end
 
--  ffi.delete(error_code)
 -- if not plan_output then
 --   plan_output = moveit.Plan()
 --   status = f.plan(self.o, plan_output:cdata())
 -- end
  
  
  return status, plan_output
end

function MoveGroup:attachObject(object, link)
  return f.attachObject(object, link or ffi.NULL)
end

function MoveGroup:detachObject(object)
  return f.detachObject(object)
end

function MoveGroup:stop()
  f.stop(self.o)
end

function MoveGroup:startStateMonitor(wait)
  return f.startStateMonitor(self.o, wait or 1.0)
end

function MoveGroup:getCurrentState()
  return moveit.RobotState(f.getCurrentState(self.o))
end

function MoveGroup:getCurrentPose_Tensor(end_effector_link, output)
  output = output or torch.DoubleTensor()
  f.getCurrentPose_Transform(self.o, end_effector_link or ffi.NULL, output:cdata())
  return output
end

function MoveGroup:getCurrentPose_StampedTransform(end_effector_link, output)
  output = output or tf.StampedTransform()
  f.getCurrentPose_StampedTransform(self.o, end_effector_link or ffi.NULL, output:cdata())
  return output
end

function MoveGroup:getCurrentPose(end_effector_link, output)
  output = output or tf.Transform()
  f.getCurrentPose(self.o, end_effector_link or ffi.NULL, output:cdata())
  return output
end

function MoveGroup:pick(object)
	--object needs to be string
	print(object)
  return f.pick(self.o,object)
end
