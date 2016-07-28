--- LUA wrapper for moveit PlanningSceneInterface environment
-- dependency to tourch.ros
-- @classmod PlanningSceneInterface
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

---Add a solit box as a collision object into the planning scene.
--@tparam string id
--@tparam number w
--@tparam number h
--@tparam number t
--@tparam tf.Transform pose
function PlanningSceneInterface:addBox(id, w, h, t, pose)
  local obj = moveit.CollisionObject()
  obj:setId(id)
  local d = torch.DoubleTensor({w,h,t})
  obj:addPrimitive(moveit.SolidPrimitiveType.BOX, d, pose)
  self:addCollisionObject(obj)
end


--- Add a cone as collision element
--@tparam string id
--@tparam number radius radius of the cone at the lower position
--@tparam number height height of the cone
--@tparam tf.Transform pose
function PlanningSceneInterface:addCone(id, radius, height, pose)
  local obj = moveit.CollisionObject()
  obj:setId(id)
  local d = torch.DoubleTensor({radius, height})
  obj:addPrimitive(moveit.SolidPrimitiveType.CONE, d, pose)
  self:addCollisionObject(obj)  
end


---Add a solit sphere as a collision object into the planning scene.
--@tparam string id
--@tparam number radius
--@tparam tf.Transform pose
function PlanningSceneInterface:addSphere(id, radius, pose)
  local obj = moveit.CollisionObject()
  obj:setId(id)
  local d = torch.DoubleTensor({ radius })
  obj:addPrimitive(moveit.SolidPrimitiveType.SPHERE, d, pose)
  self:addCollisionObject(obj)
end

---Add a solit plane as a collision object into the planning scene.
--ax + by + cz + d = 0
--@tparam string id
--@tparam number a
--@tparam number b
--@tparam number c
--@tparam number d
--@tparam tf.Transform pose
function PlanningSceneInterface:addPlane(id, a, b, c, d, pose)
  local obj = moveit.CollisionObject()
  obj:setId(id)
  obj:addPlane({a,b,c,d}, pose)
  self:addCollisionObject(obj)
end

---Add a collision object.
--@tparam moveit.CollisionObject collision_object
function PlanningSceneInterface:addCollisionObject(collision_object)
  f.addCollisionObject(self.o, collision_object:cdata())
end

---function addCollisionObject
--Remove a collision object
--@tparam table object_ids as strings
function PlanningSceneInterface:removeCollisionObjects(object_ids)
  local ids = std.StringVector(object_ids)
  f.removeCollisionObjects(self.o, ids:cdata())
end

---Get the names of all known objects in the world. 
--If with_type is set to true, only return objects that have a known type.
--@tparam[opt] boolean with_type
--@treturn std.StringVector
function PlanningSceneInterface:getKnownObjectNames(with_type)
  local result = std.StringVector()
  f.getKnownObjectNames(self.o, with_type or false, result:cdata())
  return result
end

---Get the names of known objects in the world that are located within a bounding region (specified in the frame reported by getPlanningFrame()).
--If with_type is set to true, only return objects that have a known type.
--@tparam number minx
--@tparam number miny
--@tparam number minz
--@tparam number maxx
--@tparam number maxy
--@tparam number maxz
--@tparam[opt=false] boolean with_type
--@tparam[opt] ?std.StringVector|list types
--@treturn std.StringVector()
function PlanningSceneInterface:getKnownObjectNamesInROI(minx, miny, minz, maxx, maxy, maxz, with_type, types)
  if types and not torch.isTypeOf(types, std.StringVector) then
    types = std.StringVector(types)
  end
  local result = std.StringVector()
  f.getKnownObjectNamesInROI(self.o, minx, miny, minz, maxx, maxy, maxz, with_type or false, utils.cdata(types), result:cdata())
  return result
end

---Get poses of objects specified in object_ids
--@tparam[opt] ?std.StringVector|list object_ids
--@treturn boolean
--@treturn torch.DoubleTensor
function PlanningSceneInterface:getObjectPoses(object_ids)
  if not torch.isTypeOf(object_ids, std.StringVector) then
    object_ids = std.StringVector(object_ids)
  end
  local found = std.StringVector()
  local found_poses = torch.DoubleTensor()
  f.getObjectPoses(self.o, object_ids:cdata(), found:cdata(), found_poses:cdata())
  return found, found_poses
end
