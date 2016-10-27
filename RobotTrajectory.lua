--- LUA wrapper for moveit planning environment
-- dependency to tourch.ros
-- @classmod RobotTrajectory

local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
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
    "getWayPoint",
    "setWayPointDurationFromPrevious",
    "addSuffixWayPoint",
    "addPrefixWayPoint",
    "insertWayPoint",
    "append",
    "swap",
    "clear",
    "getAverageSegmentDuration",
    "reverse",
    "unwind",
    "findWayPointIndicesForDurationAfterStart",
    "getRobotTrajectoryMsg",
    "setRobotTrajectoryMsg",
    "getWayPoint",
    "getLastWayPoint",
    "getFirstWayPoint",
    "toTensor"
  }
  f = utils.create_method_table("moveit_RobotTrajectory_", RobotTrajectory_method_names)
end

init()

function RobotTrajectory:__init(kinematic_model, group)
  self.o = f.new(kinematic_model:cdata(), group)
  self.moveit_msgs_RobotStateSpec = ros.get_msgspec('moveit_msgs/RobotState')
  self.moveit_msgs_RobotTrajectory = ros.get_msgspec('moveit_msgs/RobotTrajectory')
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

function RobotTrajectory:setWayPointDurationFromPrevious(index, value)
  f.setWayPointDurationFromPrevious(self.o,index,value)
end

function RobotTrajectory:addSuffixWayPoint(state, dt)
  state = state or moveit.RobotState.createEmpty()
  f.addSuffixWayPoint(self.o,utils.cdata(state),dt)
end

function RobotTrajectory:addPrefixWayPoint(state, dt)
  state = state or moveit.RobotState.createEmpty()
  f.addPrefixWayPoint(self.o,utils.cdata(state),dt)
end

function RobotTrajectory:insertWayPoint(index,state, dt)
  state = state or moveit.RobotState.createEmpty()
  f.insertWayPoint(self.o,index, utils.cdata(state),dt)
end

function RobotTrajectory:append(state, dt)
  state = state or moveit.RobotState.createEmpty()
  return f.append(self.o, utils.cdata(state),dt)
end

function RobotTrajectory:swap(other)
  return f.swap(self.o, utils.cdata(other))
end

function RobotTrajectory:clear()
  return f.clear(self.o)
end

function RobotTrajectory:getAverageSegmentDuration()
  return f.getAverageSegmentDuration(self.o)
end

function RobotTrajectory:reverse()
  f.reverse(self.o)
end

function RobotTrajectory:unwind()
  f.unwind(self.o)
end

function RobotTrajectory:findWayPointIndicesForDurationAfterStart(duration, before, after, blend)
  f.findWayPointIndicesForDurationAfterStart(self.o,duration, before, after, blend)
  return duration, before, after, blend
end

function RobotTrajectory:getRobotTrajectoryMsg(output)
  local msg_bytes = torch.ByteStorage()
  f.getRobotTrajectoryMsg(self.o, msg_bytes:cdata())
  local msg = output or ros.Message(self.moveit_msgs_RobotTrajectory, true)
  msg:deserialize(msg_bytes)
  return msg
end

function RobotTrajectory:setRobotTrajectoryMsg(reference_state, input)
  if torch.isTypeOf(input, ros.Message) then
    local msg_bytes = input:serialize()
    msg_bytes:shrinkToFit()
    f.setRobotTrajectoryMsg(self.o, reference_state, msg_bytes.storage:cdata())
  end
end

function RobotTrajectory:getWayPoint(index,output)
  output = output or moveit.RobotState.createEmpty()
  f.getWayPoint(self.o,index, output:cdata())
  return output
end

function RobotTrajectory:getLastWayPoint(output)
  output = output or moveit.RobotState.createEmpty()
  f.getLastWayPoint(self.o,output:cdata())
  return output
end

function RobotTrajectory:getFirstWayPoint(output)
  output = output or moveit.RobotState.createEmpty()
  f.getFirstWayPoint(self.o, output:cdata())
  return output
end

function RobotTrajectory:toTensor()
  local count = self:getWayPointCount()
  local waypoints
  local robotState
  if count > 0 then
    local tmp = self:getWayPoint(1):getVariablePositions()
    waypoints = torch.zeros(tmp:size(1),count)
    for i = 0,count-1 do
      robotState = self:getWayPoint(i)
      waypoints[{{},i+1}]:copy(robotState:getVariablePositions())
    end
  end
  return waypoints
end
