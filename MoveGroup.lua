local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'

local MoveGroup = torch.class('moveit.MoveGroup', moveit)

local f

function init()
  local MoveGroup_method_names = {
    "new",
    "delete",
    "getName",
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
    "setOrientationTarget",
    "setRPYTarget",
    "setPoseTarget_Tensor",
    "setPoseTarget_Pose",
    "setPoseTarget_PoseStamped",
    "setEndEffectorLink",
    "getEndEffectorlink",
    "clearPoseTarget",
    "clearPoseTargets",
    "move",
    "stop"
  }

  f = utils.create_method_table("moveit_MoveGroup_", MoveGroup_method_names)
  print(f)
end

init()

function MoveGroup:__init(name)
  self.f = f
  self.o = f.new(name)
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
  f.setPositionTarget(self.o, x, y, z, end_effector_link or ffi.NULL)
end

function MoveGroup:setOrientationTarget(x, y, z, w, end_effector_link)
  f.setOrientationTarget(self.o, x, y, z, w, end_effector_link or ffi.NULL)
end

function MoveGroup:setRPYTarget(roll, pitch, yaw, end_effector_link)
  f.setRPYTarget(self.o, roll, pitch, yaw, end_effector_link or ffi.NULL)
end

function MoveGroup:setPoseTarget(target)
  if torch.isTensor(target) then
    f.setPoseTarget_Tensor(self.o, target:cdata())
  end
  --f.setPoseTarget_Pose(self.o)
  --f.setPoseTarget_PoseStamped(self.o)
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

function MoveGroup:move()
  return f.move(self.o)
end

function MoveGroup:stop()
  f.stop(self.o)
end
