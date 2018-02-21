local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local ros = require 'ros'
local std = ros.std

--- LUA wrapper for moveit RobotModel environment
-- dependency to tourch.ros
-- @classmod RobotModel
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
    "getActiveJointNames",
    "getVariableNames",
    "getVariableIndex",
    "getGroupJointNames",
    "getAttachedEndEffectorNames",
    "getGroupEndEffectorTipNames",
    "getGroupEndEffectorName",
    "getEndEffectorLinkName",
    "getVariableBounds"
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

---Get complete set of End Effector names defined in srdf.
--@tparam[opt] moveit.Strings names
--@treturn moveit.Strings
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
  assert(group_name, '[nil-error] group name needs to be set')
  assert(torch.type(group_name) == 'string', string.format('[type-error] group name need to be a [string] but is type of [%s]', torch.type(group_name)))
  strings = strings or std.StringVector()
  f.getJointModelSubGroupNames(self.o, group_name, strings:cdata())
  return strings
end

function RobotModel:getJointModelNames(strings)
  strings = strings or std.StringVector()
  f.getJointModelNames(self.o, strings:cdata())
  return strings
end

---Get the joint names of the variables that make up this state without mimic or fixed joints.
--@tparam[opt] moveit.Strings names
--@treturn moveit.Strings
function RobotModel:getActiveJointNames(strings)
  strings = strings or std.StringVector()
  f.getActiveJointNames(self.o, strings:cdata())
  return strings
end

---Get the joint names of the variables that make up this state with mimic.
--@tparam[opt] moveit.Strings names
--@treturn moveit.Strings
function RobotModel:getVariableNames(strings)
  strings = strings or std.StringVector()
  f.getVariableNames(self.o, strings:cdata())
  return strings
end

function RobotModel:getVariableIndex(name)
  return f.getVariableIndex(self.o,name) + 1 --compensate c++ vslua indexing
end

function RobotModel:getGroupJointNames(group_name, strings)
  assert(group_name, '[nil-error] group name needs to be set')
  assert(torch.type(group_name) == 'string', string.format('[type-error] group name need to be a [string] but is type of [%s]', torch.type(group_name)))
  strings = strings or std.StringVector()
  f.getGroupJointNames(self.o, group_name, strings:cdata())
  return strings
end

function RobotModel:getEndEffectorNames(strings)
  strings = strings or std.StringVector()
  f.getEndEffectorNames(self.o, strings:cdata())
  return strings
end

function RobotModel:getAttachedEndEffectorNames(group_name, strings)
  strings = strings or std.StringVector()
  assert(group_name, '[nil-error] group name needs to be set')
  assert(torch.type(group_name) == 'string', string.format('[type-error] group name need to be a [string] but is type of [%s]', torch.type(group_name)))
  f.getAttachedEndEffectorNames(self.o, group_name, strings:cdata())
  print('debug: ', strings)
  if strings then
    return strings
  else
    return std.StringVector()
  end
end

function RobotModel:getGroupEndEffectorTipNames(group_name, strings)
  strings = strings or std.StringVector()
  local suc = f.getGroupEndEffectorTipNames(self.o, group_name, strings:cdata())
  return strings, suc
end

function RobotModel:getGroupEndEffectorName(group_name)
  assert(group_name, '[nil-error] group name needs to be set')
  assert(torch.type(group_name) == 'string', string.format('[type-error] group name need to be a [string] but is type of [%s]', torch.type(group_name)))
  local output = std.String()
  local suc = f.getGroupEndEffectorName(self.o, group_name, output:cdata())
  if suc then
    return output:get(), suc
  else
    return '', suc
  end
end

function RobotModel:getEndEffectorLinkName(eef_name)
  assert(eef_name, '[nil-error] group name needs to be set')
  assert(torch.type(eef_name) == 'string', string.format('[type-error] eef_name need to be a [string] but is type of [%s]', torch.type(eef_name)))
  local output = std.String()
  local suc = f.getEndEffectorLinkName(self.o, eef_name, output:cdata())
  if suc then
    return output:get(), suc
  else
    return '', suc
  end
end

function RobotModel:getVariableBounds()
  local pos_lim = torch.DoubleTensor()
  local vel_lim = torch.DoubleTensor()
  local acc_lim = torch.DoubleTensor()
  f.getVariableBounds(self.o, pos_lim:cdata(),vel_lim:cdata(), acc_lim:cdata())
  return pos_lim, vel_lim, acc_lim
end
