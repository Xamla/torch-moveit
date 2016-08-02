local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local ros = require 'ros'
local std = ros.std

local RobotModelLoader = torch.class('moveit.RobotModelLoader', moveit)

local f

function init()
  local RobotModelLoader_method_names = {
    "new",
    "delete",
    "release",
    "getModel",
    "getRobotDescription"
  }
  f = utils.create_method_table("moveit_RobotModelLoader_", RobotModelLoader_method_names)
end

init()

function RobotModelLoader:__init(robot_description, load_kinematics_solvers)
  self.o = f.new(robot_description or "robot_description", load_kinematics_solvers or true)
end

function RobotModelLoader:cdata()
  return self.o
end

function RobotModelLoader:release()
  f.release(self.o)
end

function RobotModelLoader:getModel()
  local model = moveit.RobotModel()
  f.getModel(self.o, model:cdata())
  return model
end

function RobotModelLoader:getRobotDescription()
  return ffi.string(f.getRobotDescription(self.o))
end
