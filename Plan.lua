local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'

local Plan = torch.class('moveit.Plan', moveit)

local f

function init()
  local Plan_method_names = {
    "new",
    "delete",
    "release",
    "getPlanningTime"
  }
  f = utils.create_method_table("moveit_Plan_", Plan_method_names)
end

init()

function Plan:__init()
  self.o = f.new()
end

function Plan:cdata()
  return self.o
end

function Plan:release()
  f.release(self.o)
end

function Plan:getPlanningTime()
  return f.getPlannigTime(self.o)
end
