local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local ros = require 'ros'
local std = ros.std

local RobotTrajectory = torch.class('moveit.RobotTrajectory', moveit)

local f

function init()
  local RobotTrajectory_method_names = {
    "new",
    "delete",
    "release",
    "getGroupName"
  }
  f = utils.create_method_table("moveit_RobotTrajectory_", RobotTrajectory_method_names)
end

init()

function RobotTrajectory:__init()
  self.o = f.new()
end

function RobotTrajectory:cdata()
  return self.o
end

function RobotTrajectory:release()
  f.release(self.o)
end

function RobotTrajectory:getGroupName()
  return ffi.string(f.getName(self.o))
end
