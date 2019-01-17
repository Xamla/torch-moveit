#include "torch-moveit.h"
#include "utils.h"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetPlanningScene.h>
typedef robot_model::RobotModelPtr RobotModelPtr;
typedef planning_scene::PlanningScenePtr PlanningScenePtr;

MOVIMP(PlanningScenePtr*, PlanningScene, new)(
  RobotModelPtr *robot_model
)
{
  if (robot_model == NULL || *robot_model == NULL)
    throw MoveItWrapperException("Argument 'robot_model' must not be NULL.");
  PlanningScenePtr* ptr = new PlanningScenePtr(new planning_scene::PlanningScene(*robot_model));
  return ptr;
}

MOVIMP(void, PlanningScene, delete)(PlanningScenePtr *ptr)
{
  if (ptr)
    delete ptr;
}

MOVIMP(void, PlanningScene, release)(PlanningScenePtr *ptr)
{
  ptr->reset();
}

MOVIMP(void, PlanningScene, setCurrentState)(PlanningScenePtr *ptr, RobotStatePtr *robot_state)
{
  if (robot_state != NULL && *robot_state != NULL)
    (*ptr)->setCurrentState(**robot_state);
}

MOVIMP(RobotStatePtr *, PlanningScene, getCurrentState)(PlanningScenePtr *self)
{
  return new RobotStatePtr(new moveit::core::RobotState((*self)->getCurrentState()));
}

MOVIMP(bool, PlanningScene, checkSelfCollision)(PlanningScenePtr *ptr, RobotStatePtr *robot_state)
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  if (robot_state != NULL && *robot_state != NULL)
    (*ptr)->checkSelfCollision(collision_request, collision_result, **robot_state);
  else
    (*ptr)->checkSelfCollision(collision_request, collision_result);
  return collision_result.collision;
}

MOVIMP(bool, PlanningScene, isStateColliding)(PlanningScenePtr *ptr,RobotStatePtr *robot_state, const char *group_name, bool verbose)
{
  //TODO
  return (*ptr)->isStateColliding(**robot_state, group_name, verbose);
}

MOVIMP(bool, PlanningScene, isPathValid)(PlanningScenePtr *ptr, RobotStatePtr *start_state, RobotTrajectoryPtr *trajectory, const char *group_name, bool verbose)
{
  std::vector<std::size_t> invalid_index;
  moveit_msgs::RobotTrajectory trajectory_msg;
  (*trajectory)->getRobotTrajectoryMsg(trajectory_msg);

  moveit_msgs::RobotState robot_state_msg;
  robot_state::robotStateToRobotStateMsg(**start_state, robot_state_msg, true);

  return (*ptr)->isPathValid(robot_state_msg, trajectory_msg, group_name, verbose, &invalid_index);
}


MOVIMP(bool, PlanningScene, setPlanningSceneMsg)(PlanningScenePtr *ptr, THByteStorage *serialized_msg)
{
  moveit_msgs::PlanningScene scene_msg;
  // deserialize to moveit_msgs::RobotTrajectory message
  long storage_size = THByteStorage_size(serialized_msg);

  uint8_t *data = THByteStorage_data(serialized_msg);
  ros::serialization::IStream stream(data+ sizeof(uint32_t), static_cast<uint32_t>(storage_size-sizeof(uint32_t)));
  ros::serialization::Serializer<moveit_msgs::PlanningScene>::read(stream, scene_msg);
  return (*ptr)->setPlanningSceneMsg(scene_msg);
}


MOVIMP(bool, PlanningScene, syncPlanningScene)(PlanningScenePtr *ptr)
{
  ros::NodeHandle nodehandle;

  ros::ServiceClient client_get_scene = nodehandle.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components =
      srv.request.components.SCENE_SETTINGS |
      srv.request.components.ROBOT_STATE |
      srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS |
      srv.request.components.WORLD_OBJECT_NAMES |
      srv.request.components.WORLD_OBJECT_GEOMETRY |
      srv.request.components.OCTOMAP |
      srv.request.components.TRANSFORMS |
      srv.request.components.ALLOWED_COLLISION_MATRIX |
      srv.request.components.LINK_PADDING_AND_SCALING |
      srv.request.components.OBJECT_COLORS;
  // Make sure client is connected to server
  if (!client_get_scene.exists())
  {
    ROS_WARN("Waiting for service `/get_planning_scene` to exist.");
    client_get_scene.waitForExistence(ros::Duration(5.0));
  }
  if (!client_get_scene.exists())
  {
    ROS_ERROR("Service `/get_planning_scene` does not exist.");
    return false;
  }
  if (!client_get_scene.call(srv)){
     ROS_WARN("Failed to call service /get_planning_scene");
 }
  //return (*ptr)->usePlanningSceneMsg(srv.response.scene);
  bool suc = (*ptr)->setPlanningSceneMsg(srv.response.scene);
  if (!suc){
    ROS_ERROR("could not set planning scene from /get_planning_scene service");
  }
  return suc;
}
