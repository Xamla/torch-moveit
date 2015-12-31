local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local tf = moveit.tf

local TransformBroadcaster = torch.class('tf.TransformBroadcaster', tf)

local f

function init()
  local TransformBroadcaster_method_names = {
    "new",
    "delete",
    "sendTransform"
  }
  
  f = utils.create_method_table("tf_TransformBroadcaster_", TransformBroadcaster_method_names)
end

init()

function TransformBroadcaster:__init()
  self.o = f.new()
end

function TransformBroadcaster:sendTransform(stampedTransform)
  f.sendTransform(self.o, stampedTransform:cdata())
end
