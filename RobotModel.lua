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
    "getEndEffectorNames"
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
  return f.getEndEffectorNames(self.o, strings:cdata())
end