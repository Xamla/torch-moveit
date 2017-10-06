local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local ros = require 'ros'
local std = ros.std

local RobotModel = torch.class('moveit.RobotModel', moveit)

local f

function init()
  local RobotModel_method_names = {
    "new",
    "delete",
    "release",
    "getName",
    "getModelFrame",
    "isEmpty",
    "printModelInfo",
    "getRootJointName",
    "getEndEffectorNames",
    "getEndEffectorParentGroups",
    "getJointModelGroupNames",
    "getJointModelSubGroupNames",
    "getJointModelNames",
    "getVariableNames",
    "getVariableIndex",
    "getGroupJointNames",
    "getGroupEndEffectorNames"
  }
  f = utils.create_method_table("moveit_RobotModel_", RobotModel_method_names)
end

init()

function RobotModel:__init()
  self.o = f.new()
end

function RobotModel:cdata()
  return self.o
end

function RobotModel:release()
  f.release(self.o)
end

function RobotModel:getName()
  return ffi.string(f.getName(self.o))
end

function RobotModel:getModelFrame()
  return ffi.string(f.getModelFrame(self.o))
end

function RobotModel:isEmpty()
  return f.isEmpty(self.o)
end

function RobotModel:printModelInfo()
  local s = std.String()
  f.printModelInfo(self.o, s:cdata())
  return s
end

function RobotModel:getRootJointName()
  return ffi.string(f.getRootJointName(self.o))
end

function RobotModel:getEndEffectorNames(strings)
  strings = strings or std.StringVector()
  f.getEndEffectorNames(self.o, strings:cdata())
  return strings
end

function RobotModel:getEndEffectorParentGroups()
  local parent_move_group_names = std.StringVector()
  local move_group_link_names = std.StringVector()
  f.getEndEffectorParentGroups(self.o, parent_move_group_names:cdata(), move_group_link_names:cdata())
  return parent_move_group_names, move_group_link_names
end

function RobotModel:getJointModelGroupNames(strings)
  strings = strings or std.StringVector()
  f.getJointModelGroupNames(self.o, strings:cdata())
  return strings
end

function RobotModel:getJointModelSubGroupNames(group_name, strings)
  strings = strings or std.StringVector()
  f.getJointModelSubGroupNames(self.o, group_name, strings:cdata())
  return strings
end

function RobotModel:getJointModelNames(strings)
  strings = strings or std.StringVector()
  f.getJointModelNames(self.o, strings:cdata())
  return strings
end

function RobotModel:getVariableNames(strings)
  strings = strings or std.StringVector()
  f.getVariableNames(self.o, strings:cdata())
  return strings
end

function RobotModel:getVariableIndex(name)
  return f.getVariableIndex(self.o,name)
end

function RobotModel:getGroupJointNames(group_name, strings)
  strings = strings or std.StringVector()
  f.getGroupJointNames(self.o, group_name, strings:cdata())
  return strings
end

function RobotModel:getGroupEndEffectorNames(group_name, strings)
  strings = strings or std.StringVector()
  f.getGroupEndEffectorNames(self.o, group_name, strings:cdata())
  return strings
end
