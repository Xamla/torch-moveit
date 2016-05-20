local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local tf = ros.tf

local RobotState = torch.class('moveit.RobotState', moveit)

local f

function init()
  local RobotState_method_names = {
    "clone",
    "delete",
    "release",
    "getVariableCount",
    "getVariableNames",
    "getVariablePositions",
    "hasVelocities",
    "getVariableVelocities",
    "hasAccelerations",
    "getVariableAccelerations",
    "hasEffort",
    "getVariableEffort",
    "setToDefaultValues",
    "setToRandomPositions",
    "setFromIK"
  }
  
  f = utils.create_method_table("moveit_RobotState_", RobotState_method_names)
end

init()

function RobotState:__init(o)
  if type(o) ~= 'cdata' or ffi.typeof(o) ~= ffi.typeof('RobotStatePtr*') then
    error('RobotState object can only be initialized with existing RobotState pointer.')
  end
  self.o = o
  ffi.gc(o, f.delete)
end

function RobotState:cdata()
  return self.o
end

function RobotState:clone()
  local c = torch.factory('moveit.RobotState')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function RobotState:getVariableCount()
  return f.getVariableCount(self.o)
end

function RobotState:getVariableNames(names)
  names = names or moveit.Strings()
  f.getVariableNames(self.o, names:cdata())
  return names
end

function RobotState:getVariablePositions()
  local t = torch.DoubleTensor()
  f.getVariablePositions(self.o, t:cdata())
  return t
end

function RobotState:hasVelocities()
  return f.hasVelocities(self.o)
end

function RobotState:getVariableVelocities()
  local t = torch.DoubleTensor()
  f.getVariableVelocities(self.o, t:cdata())
  return t
end

function RobotState:hasAccelerations()
  return f.hasAccelerations(self.o)
end

function RobotState:getVariableAccelerations()
  local t = torch.DoubleTensor()
  f.getVariableAccelerations(self.o, t:cdata())
  return t
end

function RobotState:hasEffort()
  return f.hasEffort(self.o)
end

function RobotState:getVariableEffort()
  local t = torch.DoubleTensor()
  f.getVariableEffort(self.o, t:cdata())
  return t
end

function RobotState:setToDefaultValues()
  f.setToDefaultValues(self.o)
end

function RobotState:setToRandomPositions()
  f.setToRandomPositions(self.o)
end

function RobotState:setFromIK(group_id,pose_)
  local tensor = torch.DoubleTensor()
  local suc = f.setFromIK(self.o, group_id, pose_:cdata(), tensor:cdata())
  return suc, tensor
end
