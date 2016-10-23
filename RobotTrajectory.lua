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
    "findWayPointIndicesForDurationAfterStart"
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

function RobotTrajectory:setWayPointDurationFromPrevious(index, value)
  f.setWayPointDurationFromPrevious(self.o,index,value)
end

function RobotTrajectory:addSuffixWayPoint(state, dt)
  f.addSuffixWayPoint(self.o,utils.cdata(state),dt)
end

function RobotTrajectory:addPrefixWayPoint(state, dt)
  f.addPrefixWayPoint(self.o,utils.cdata(state),dt)
end

function RobotTrajectory:insertWayPoint(index,state, dt)
  f.insertWayPoint(self.o,index, utils.cdata(state),dt)
end

function RobotTrajectory:append(state, dt)
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
