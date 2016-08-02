#include "torch-moveit.h"
#include "utils.h"
#include <moveit/robot_model_loader/robot_model_loader.h>

typedef robot_model_loader::RobotModelLoaderPtr RobotModelLoaderPtr;
typedef robot_model::RobotModelPtr RobotModelPtr;

MOVIMP(RobotModelLoaderPtr*, RobotModelLoader, new)(
  const char *robot_description /*= "robot_description"*/,
  bool load_kinematics_solvers /*= true*/
)
{
   return new RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader(std::string(robot_description), load_kinematics_solvers));
}

MOVIMP(void, RobotModelLoader, delete)(RobotModelLoaderPtr *ptr)
{
  if (ptr)
    delete ptr;
}

MOVIMP(void, RobotModelLoader, release)(RobotModelLoaderPtr *ptr)
{
  ptr->reset();
}

MOVIMP(void, RobotModelLoader, getModel)(RobotModelLoaderPtr *ptr, RobotModelPtr *output)
{
  *output = (*ptr)->getModel();
}

MOVIMP(const char *, RobotModelLoader, getRobotDescription)(RobotModelLoaderPtr *ptr)
{
  return (*ptr)->getRobotDescription().c_str();
}
