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
    "setCurrentState",
    "checkSelfCollision",
    "isStateColliding",
    "isPathValid",
    "setPlanningSceneMsg",
    "syncPlanningScene"
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
  if torch.type(robot_state) ~= 'moveit.RobotState' then
    error('Argument positions of wrong type. moveit.RobotState expected.')
  end
  return f.setCurrentState(self.o, utils.cdata(robot_state))
end

function PlanningScene:checkSelfCollision(robot_state)
  if torch.type(robot_state) ~= 'moveit.RobotState' then
    error('Argument positions of wrong type. moveit.RobotState expected.')
  end
  return f.checkSelfCollision(self.o, utils.cdata(robot_state))
end

function PlanningScene:isStateColliding(group_name,robot_state, verbose)
  if torch.type(robot_state) ~= 'moveit.RobotState' then
    error('Argument positions of wrong type. moveit.RobotState expected.')
  end
  local group_name  = group_name or ""
  local verbose = verbose or false
  return f.isStateColliding(self.o,robot_state:cdata(),group_name, verbose)
end

function PlanningScene:isPathValid(start_state,trajectory, group_name,verbose)
  if torch.type(start_state) ~= 'moveit.RobotState' then
    error('Argument positions of wrong type. moveit.RobotState expected.')
  end
  if torch.type(trajectory) ~= 'moveit.RobotTrajectory' then
    error('Argument orientations of wrong type. moveit.RobotTrajectory expected.')
  end
  local group_name  = group_name or ""
  local verbose = verbose or false
  return f.isPathValid(self.o,start_state:cdata(),trajectory:cdata(),group_name, verbose)
end

function PlanningScene:setPlanningSceneMsg(input)
 if torch.isTypeOf(input, ros.Message) then
    local msg_bytes = input:serialize()
    msg_bytes:shrinkToFit()
    return f.setPlanningSceneMsg(self.o, msg_bytes.storage:cdata())
  end
end

function PlanningScene:syncPlanningScene()
  f.syncPlanningScene(self.o)
end