local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local tf = ros.tf

local CollisionObject = torch.class('moveit.CollisionObject', moveit)

local f

function init()
  local CollisionObject_method_names = {
    "new",
    "delete",
    "getId",
    "setId",
    "getFrameId",
    "setFrameId",
    "getOperation",
    "setOperation",
    "addPrimitive",
    "addPlane"
  }
  f = utils.create_method_table("moveit_CollisionObject_", CollisionObject_method_names)
end

init()

moveit.SolidPrimitiveType = {
  BOX = 1,      -- BOX_X, BOX_Y, BOX_Z
  SPHERE = 2,   -- SPHERE_RADIUS
  CYLINDER = 3, -- CYLINDER_HEIGHT, CYLINDER_RADIUS
  CONE = 4      -- CONE_HEIGHT, CONE_RADIUS
}

function CollisionObject:__init()
  self.o = f.new()
end

function CollisionObject:cdata()
  return self.o
end

function CollisionObject:getId()
  return ffi.string(f.getId(self.o))
end

function CollisionObject:setId(id)
  f.setId(self.o, id)
end

function CollisionObject:getFrameId()
  return ffi.string(f.getFrameId(self.o))
end

function CollisionObject:setFrameId(frame_id)
  f.setFrameId(self.o, frame_id)
end

function CollisionObject:getOperation()
  return f.getOperation(self.o)
end

function CollisionObject:setOperation(op)
  f.setOperation(self.o, op)
end

function CollisionObject:addPrimitive(primitive_type, dimensions, pose)
  if not torch.isTensor(dimensions) then
    dimensions = torch.DoubleTensor(dimensions)
  end
  pose = pose or tf.Transform() -- identity by default
  f.addPrimitive(self.o, primitive_type, dimensions:cdata(), pose:cdata())
end

function CollisionObject:addPlane(coefs, pose)
  if not torch.isTensor(coefs) then
    coefs = torch.DoubleTensor(coefs)
  end
  pose = pose or tf.Transform() -- identity by default
  f.addPlane(self.o, coefs:cdata(), pose:cdata())
end
