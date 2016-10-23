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
    "empty",
    "getGroupName",
    "setGroupName",
    "getWayPointCount",
    "getWayPoint"
  }
  f = utils.create_method_table("moveit_RobotTrajectory_", RobotTrajectory_method_names)
end

init()

function RobotTrajectory:__init(kinematic_model, group)
  self.o = f.new(kinematic_model:cdata(), group)
end

function RobotTrajectory:cdata()
  return self.o
end

function RobotTrajectory:release()
  f.release(self.o)
end

function RobotTrajectory:empty()
  return f.empty(self.o)
end

function RobotTrajectory:getGroupName()
  return ffi.string(f.getGroupName(self.o))
end

function RobotTrajectory:setGroupName(name)
  return f.setGroupName(self.o,name)
end

function RobotTrajectory:getWayPointCount()
  return f.getWayPointCount(self.o)
end

function RobotTrajectory:getWayPoint(index)
  return f.getWayPoint(self.o, index)
end