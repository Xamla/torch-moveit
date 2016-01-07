local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'

local PlanningSceneInterface = torch.class('moveit.PlanningSceneInterface', moveit)

local f

function init()
  local PlanningSceneInterface_method_names = {
    "new",
    "delete",
    "addCollisionObject",
    "removeCollisionObjects",
    "getKnownObjectNames",
    "getKnownObjectNamesInROI",
    "getObjectPoses"
  }
  f = utils.create_method_table("moveit_PlanningSceneInterface_", PlanningSceneInterface_method_names)
end

init()

function PlanningSceneInterface:__init()
  self.o = f.new()
end

function PlanningSceneInterface:cdata()
  return self.o
end

function PlanningSceneInterface:addBox(id, w, h, t, pose)
  local obj = moveit.CollisionObject()
  obj:setId(id)
  local d = torch.DoubleTensor({w,h,t})
  obj:addPrimitive(moveit.SolidPrimitiveType.BOX, d, pose)
  self:addCollisionObject(obj)
end

function PlanningSceneInterface:addSphere(id, radius, pose)
  local obj = moveit.CollisionObject()
  obj:setId(id)
  local d = torch.DoubleTensor({ radius })
  obj:addPrimitive(moveit.SolidPrimitiveType.SPHERE, d, pose)
  self:addCollisionObject(obj)
end

-- ax + by + cz + d = 0
function PlanningSceneInterface:addPlane(id, a, b, c, d, pose)
  local obj = moveit.CollisionObject()
  obj:setId(id)
  obj:addPlane({a,b,c,d}, pose)
  self:addCollisionObject(obj)
end

function PlanningSceneInterface:addCollisionObject(collision_object)
  f.addCollisionObject(self.o, collision_object:cdata())
end

function PlanningSceneInterface:removeCollisionObjects(object_ids)
  local ids = std.StringVector(object_ids)
  f.removeCollisionObjects(self.o, ids:cdata())
end

function PlanningSceneInterface:getKnownObjectNames(with_type)
  local result = std.StringVector()
  f.getKnownObjectNames(self.o, with_type or false, result:cdata())
  return result
end

function PlanningSceneInterface:getKnownObjectNamesInROI(minx, miny, minz, maxx, maxy, maxz, with_type, types)
  if types and not torch.isTypeOf(types, std.StringVector) then
    types = std.StringVector(types)
  end
  local result = std.StringVector()
  f.getKnownObjectNamesInROI(self.o, minx, miny, minz, maxx, maxy, maxz, with_type or false, utils.cdata(types), result:cdata())
  return result
end

function PlanningSceneInterface:getObjectPoses(object_ids)
  if not torch.isTypeOf(object_ids, std.StringVector) then
    object_ids = std.StringVector(object_ids)
  end
  local found = std.StringVector()
  local found_poses = torch.DoubleTensor()
  f.getObjectPoses(self.o, object_ids:cdata(), found:cdata(), found_poses:cdata())
  return found, found_poses
end
