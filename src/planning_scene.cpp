#include "torch-moveit.h"
#include "utils.h"
#include <moveit/planning_scene/planning_scene.h>

typedef robot_model::RobotModelPtr RobotModelPtr;
typedef planning_scene::PlanningScenePtr PlanningScenePtr;

MOVIMP(PlanningScenePtr*, PlanningScene, new)(
  RobotModelPtr *robot_model
)
{
  if (robot_model == NULL || *robot_model == NULL)
    throw MoveItWrapperException("Argument 'robot_model' must not be NULL.");

  return new PlanningScenePtr(new planning_scene::PlanningScene(*robot_model));
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
