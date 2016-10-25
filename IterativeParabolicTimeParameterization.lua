--- LUA wrapper for moveit planning environment
-- dependency to tourch.ros
-- @classmod IterativeParabolicTimeParameterization
local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local tf = ros.tf

local IterativeParabolicTimeParameterization = torch.class('moveit.IterativeParabolicTimeParameterization', moveit)

local f

function init()
  local IterativeParabolicTimeParameterization_method_names = {
    "new",
    "delete",
    "release",
    "computeTimeStamps"
  }

  f = utils.create_method_table("moveit_IterativeParabolicTimeParameterization_", IterativeParabolicTimeParameterization_method_names)
end

init()

--- Init function.
-- Specify the group name for which to construct this commander instance.
-- Throws an exception if there is an initialization error.
-- @tparam string name of the kinematic move group
function IterativeParabolicTimeParameterization:__init()
  self.o = f.new()
end

function IterativeParabolicTimeParameterization:release()
  f.release(self.o)
end

function IterativeParabolicTimeParameterization:cdata()
  return self.o
end

function IterativeParabolicTimeParameterization:computeTimeStamps(robotTrajectory)
  f.computeTimeStamps(self.o,robotTrajectory:cdata())
end
