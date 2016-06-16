local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'

----------------------
-- LUA wrapper for moveit PlanningSceneInterface environment
-- dependency to tourch.ros
-- module: MoveGroup
--
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

---function addBox
--Add a solit box as a collision object into the planning scene
--@tparam string
--@tparam number
--@tparam number
--@tparam number
--@tparam tf.Transform
function PlanningSceneInterface:addBox(id, w, h, t, pose)
  local obj = moveit.CollisionObject()
  obj:setId(id)
  local d = torch.DoubleTensor({w,h,t})
  obj:addPrimitive(moveit.SolidPrimitiveType.BOX, d, pose)
  self:addCollisionObject(obj)
end

---function addSphere
--Add a solit sphere as a collision object into the planning scene
--@tparam string
--@tparam number
--@tparam tf.Transform
function PlanningSceneInterface:addSphere(id, radius, pose)
  local obj = moveit.CollisionObject()
  obj:setId(id)
  local d = torch.DoubleTensor({ radius })
  obj:addPrimitive(moveit.SolidPrimitiveType.SPHERE, d, pose)
  self:addCollisionObject(obj)
end

---function addSphere
--Add a solit sphere as a collision object into the planning scene
--ax + by + cz + d = 0
--@tparam string
--@tparam number
--@tparam number
--@tparam number
--@tparam number
--@tparam tf.Transform
function PlanningSceneInterface:addPlane(id, a, b, c, d, pose)
  local obj = moveit.CollisionObject()
  obj:setId(id)
  obj:addPlane({a,b,c,d}, pose)
  self:addCollisionObject(obj)
end

---function addCollisionObject
--Add a collision object
--@tparam moveit.CollisionObject()
function PlanningSceneInterface:addCollisionObject(collision_object)
  f.addCollisionObject(self.o, collision_object:cdata())
end

---function addCollisionObject
--Remove a collision object
--@tparam table object ids as strings
function PlanningSceneInterface:removeCollisionObjects(object_ids)
  local ids = std.StringVector(object_ids)
  f.removeCollisionObjects(self.o, ids:cdata())
end

---function getKnownObjectNames
--Get the names of all known objects in the world. If with_type is set to true, only return objects that have a known type.
--@tparam[opt] bool
--@treturn std.StringVector()
function PlanningSceneInterface:getKnownObjectNames(with_type)
  local result = std.StringVector()
  f.getKnownObjectNames(self.o, with_type or false, result:cdata())
  return result
end

---function getKnownObjectNamesInROI
--Get the names of known objects in the world that are located within a bounding region (specified in the frame reported by getPlanningFrame()).
--If with_type is set to true, only return objects that have a known type.
--@tparam number
--@tparam number
--@tparam number
--@tparam number
--@tparam number
--@tparam number
--@tparam[opt=false] bool
--@tparam[opt] ?std.StringVector()|list
--@treturn std.StringVector()
function PlanningSceneInterface:getKnownObjectNamesInROI(minx, miny, minz, maxx, maxy, maxz, with_type, types)
  if types and not torch.isTypeOf(types, std.StringVector) then
    types = std.StringVector(types)
  end
  local result = std.StringVector()
  f.getKnownObjectNamesInROI(self.o, minx, miny, minz, maxx, maxy, maxz, with_type or false, utils.cdata(types), result:cdata())
  return result
end

---function getObjectPoses
--Get poses of objects specified in object_ids
--@tparam[opt] ?std.StringVector()|list
--@treturn bool
--@treturn torch.DoubleTensor()
function PlanningSceneInterface:getObjectPoses(object_ids)
  if not torch.isTypeOf(object_ids, std.StringVector) then
    object_ids = std.StringVector(object_ids)
  end
  local found = std.StringVector()
  local found_poses = torch.DoubleTensor()
  f.getObjectPoses(self.o, object_ids:cdata(), found:cdata(), found_poses:cdata())
  return found, found_poses
end
