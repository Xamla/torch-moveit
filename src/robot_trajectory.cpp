#include "torch-moveit.h"
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "utils.h"

typedef robot_model::RobotModelPtr RobotModelPtr;
typedef robot_trajectory::RobotTrajectoryPtr RobotTrajectoryPtr;

MOVIMP(RobotTrajectoryPtr*, RobotTrajectory, new)(RobotModelPtr* robot_model, const char *group)
{
   return new RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(*robot_model, group));
}

MOVIMP(void, RobotTrajectory, delete)(RobotTrajectoryPtr *ptr)
{
  if (ptr)
    delete ptr;
}

MOVIMP(void, RobotTrajectory, release)(RobotTrajectoryPtr *ptr)
{
  ptr->reset();
}

MOVIMP(const char *, RobotTrajectory, getGroupName)(RobotTrajectoryPtr *ptr)
{
  return (*ptr)->getGroupName().c_str();
}

MOVIMP(bool, RobotTrajectory, empty)(RobotTrajectoryPtr *ptr)
{
  return (*ptr)->empty();
}

