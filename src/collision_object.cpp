#include "torch-moveit.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "utils.h"

typedef std::shared_ptr<moveit_msgs::CollisionObject> CollisionObjectPtr;

MOVIMP(CollisionObjectPtr*, CollisionObject, new)()
{
  CollisionObjectPtr *p = new CollisionObjectPtr(new moveit_msgs::CollisionObject());
  (*p)->operation = moveit_msgs::CollisionObject::ADD;
  return p;
}

MOVIMP(void, CollisionObject, delete)(CollisionObjectPtr *ptr)
{
  if (ptr)
    delete ptr;
}

MOVIMP(const char *, CollisionObject, getId)(CollisionObjectPtr *self)
{
  return (*self)->id.c_str();
}

MOVIMP(void, CollisionObject, setId)(CollisionObjectPtr *self, const char *id)
{
  (*self)->id = id;
}

MOVIMP(const char *, CollisionObject, getFrameId)(CollisionObjectPtr *self)
{
  return (*self)->header.frame_id.c_str();
}

MOVIMP(void, CollisionObject, setFrameId)(CollisionObjectPtr *self, const char *id)
{
  (*self)->header.frame_id = id;
}

MOVIMP(int, CollisionObject, getOperation)(CollisionObjectPtr *self)
{
  return (*self)->operation;
}

MOVIMP(void, CollisionObject, setOperation)(CollisionObjectPtr *self, int operation)
{
  (*self)->operation = static_cast< moveit_msgs::CollisionObject::_operation_type>(operation);
}

MOVIMP(void, CollisionObject, addPrimitive)(CollisionObjectPtr *self, int type, THDoubleTensor *dimensions, tf::Transform *transform)
{
  shape_msgs::SolidPrimitive primitive;
  primitive.type = type;
  Tensor2vector(dimensions, primitive.dimensions);
  (*self)->primitives.push_back(primitive);
  
  geometry_msgs::Pose pose;   // convert transform to pose msg
  poseTFToMsg(*transform, pose);
  (*self)->primitive_poses.push_back(pose);
}

MOVIMP(void, CollisionObject, addPlane)(CollisionObjectPtr *self, THDoubleTensor *coefs, tf::Transform *transform)
{
  shape_msgs::Plane plane;
  for (int i = 0; i < 4; ++i)
    plane.coef[i] = THDoubleTensor_get1d(coefs, i);  
  (*self)->planes.push_back(plane);
  
  geometry_msgs::Pose pose;   // convert transform to pose msg
  poseTFToMsg(*transform, pose);
  (*self)->plane_poses.push_back(pose);
}
