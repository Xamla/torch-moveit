#include "torch-moveit.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include "utils.h"

typedef std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningSceneInterfacePtr;
typedef std::shared_ptr<moveit_msgs::CollisionObject> CollisionObjectPtr;

MOVIMP(PlanningSceneInterfacePtr*, PlanningSceneInterface, new)()
{
   return new PlanningSceneInterfacePtr(new moveit::planning_interface::PlanningSceneInterface());
}

MOVIMP(void, PlanningSceneInterface, delete)(PlanningSceneInterfacePtr *ptr)
{
  if (ptr)
    delete ptr;
}

MOVIMP(void, PlanningSceneInterface, addCollisionObject)(PlanningSceneInterfacePtr *self, CollisionObjectPtr *obj)
{
  (*self)->addCollisionObjects(std::vector<moveit_msgs::CollisionObject>(1, **obj));
}

MOVIMP(void, PlanningSceneInterface, removeCollisionObjects)(PlanningSceneInterfacePtr *self, std::vector<std::string> *object_ids)
{
  (*self)->removeCollisionObjects(*object_ids);
}

MOVIMP(void, PlanningSceneInterface, getKnownObjectNames)(PlanningSceneInterfacePtr *self, bool with_type, std::vector<std::string> *result)
{
  *result = (*self)->getKnownObjectNames(with_type);
}

MOVIMP(void, PlanningSceneInterface, getKnownObjectNamesInROI)(PlanningSceneInterfacePtr *self, double minx, double miny, double minz, double maxx, double maxy, double maxz, bool with_type, 
  std::vector<std::string>* types, std::vector<std::string>* result)
{
 if (types)
   *result = (*self)->getKnownObjectNamesInROI(minx, miny, minz, maxx, maxy, maxz, with_type, *types);
 else
   *result = (*self)->getKnownObjectNamesInROI(minx, miny, minz, maxx, maxy, maxz, with_type);
}

MOVIMP(void, PlanningSceneInterfacePtr, getObjectPoses)(PlanningSceneInterfacePtr *self, std::vector<std::string> *object_ids, std::vector<std::string> *found, THDoubleTensor *found_poses)
{
  std::map<std::string, geometry_msgs::Pose> poses = (*self)->getObjectPoses(*object_ids);
  found->clear();
  
  THDoubleTensor_resize3d(found_poses, poses.size(), 4, 4);
  THDoubleTensor* found_poses_ = THDoubleTensor_newContiguous(found_poses);
  double *data = THDoubleTensor_data(found_poses_);
  
  // allocate space in result result
  for (std::map<std::string, geometry_msgs::Pose>::const_iterator i = poses.begin(); i != poses.end(); ++i)
  {
    const std::string& id = i->first;
    const geometry_msgs::Pose& pose = i->second;
    found->push_back(id);
    
    // convert to eigen matrix
    Eigen::Affine3d e;
    tf::poseMsgToEigen(pose, e);
    
    // copy data
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > mapping(data);
    mapping = e.matrix();
    
    data = data + 16;
  }
  
  THDoubleTensor_freeCopyTo(found_poses_, found_poses);
}
