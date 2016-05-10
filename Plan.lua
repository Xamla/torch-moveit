local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'

local Plan = torch.class('moveit.Plan', moveit)

local f

function init()
  local Plan_method_names = {
    "new",
    "delete",
    "release",
    "getStartStateMsg",
    "getTrajectoryMsg",
    "getPlanningTime"
  }
  f = utils.create_method_table("moveit_Plan_", Plan_method_names)
end

init()

function Plan:__init()
  self.o = f.new()
  self.moveit_msgs_RobotStateSpec = ros.get_msgspec('moveit_msgs/RobotState')
  self.moveit_msgs_RobotTrajectory = ros.get_msgspec('moveit_msgs/RobotTrajectory')
end

function Plan:cdata()
  return self.o
end

function Plan:release()
  f.release(self.o)
end

function Plan:getStartStateMsg(result)
  local msg_bytes = torch.ByteStorage()
  f.getStartStateMsg(self.o, msg_bytes:cdata())
  local msg = result or ros.Message(self.moveit_msgs_RobotStateSpec, true)
  msg:deserialize(msg_bytes)
  return msg
end

function Plan:getTrajectoryMsg(result)
  local msg_bytes = torch.ByteStorage()
  f.getTrajectoryMsg(self.o, msg_bytes:cdata())
  local msg = result or ros.Message(self.moveit_msgs_RobotTrajectory, true)
  msg:deserialize(msg_bytes)
  return msg
end

function Plan:getPlanningTime()
  return f.getPlannigTime(self.o)
end
