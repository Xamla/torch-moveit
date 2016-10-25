local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local ros = require 'ros'
local std = ros.std

local PlanningScene = torch.class('moveit.PlanningScene', moveit)

local f

function init()
  local PlanningScene_method_names = {
    "new",
    "delete",
    "release",
    "checkSelfCollision",
    "isStateColliding"
  }
  f = utils.create_method_table("moveit_PlanningScene_", PlanningScene_method_names)
end

init()

function PlanningScene:__init(robot_model)
  self.o = f.new(robot_model:cdata())
end

function PlanningScene:cdata()
  return self.o
end

function PlanningScene:release()
  f.release(self.o)
end

function PlanningScene:setCurrentState(robot_state)
  return f.setCurrentState(self.o, utils.cdata(robot_state))
end

function PlanningScene:checkSelfCollision(robot_state)
  return f.checkSelfCollision(self.o, utils.cdata(robot_state))
end

function PlanningScene:isStateColliding(group_name, verbose)
  local group_name  = group_name or ""
  local verbose = verbose or false
  return f.isStateColliding(group_name, verbose)
end