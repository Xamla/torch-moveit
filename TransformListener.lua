local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local std = moveit.std
local tf = moveit.tf

local TransformListener = torch.class('tf.TransformListener', tf)

local f

function init()
  local TransformListener_method_names = {
    "new",
    "delete",
    "clear",
    "getFrameStrings",
    "lookupTransform",
    "waitForTransform",
    "canTransform",
    "resolve"
  }
  
  f = utils.create_method_table("tf_TransformListener_", TransformListener_method_names)
end

init()

function TransformListener:__init()
  self.o = f.new()
end

function TransformListener:clear()
  f.clear(self.o)
end

function TransformListener:getFrameStrings()
  local result = std.StringVector()
  f.getFrameStrings(self.o, result:cdata())
  return result
end

function TransformListener:lookupTransform(target_frame, source_frame, time, result)
  result = result or tf.StampedTransform()
  f.lookupTransform(self.o, target_frame, source_frame, time:cdata(), result:cdata())
  return result
end

function TransformListener:waitForTransform(target_frame, source_frame, time, timeout)
  local error_msg = std.String()
  local success = f.waitForTransform(self.o, target_frame, source_frame, time:cdata(), timeout:cdata(), error_msg:cdata())
  return success, error_msg:get()
end

function TransformListener:canTransform(target_frame, source_frame, time)
  local error_msg = std.String()
  local success = f.canTransform(self.o, target_frame, source_frame, time:cdata(), error_msg:cdata())
  return success, error_msg:get()
end

function TransformListener:resolve(frame_name)
  local name = std.String()
  f.resolve(self.o, frame_name, name:cdata())
  return name:get()
end
