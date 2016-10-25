#include "torch-moveit.h"
#include "utils.h"

MOVIMP(IptPtr*, IterativeParabolicTimeParameterization, new)()
{
  try
    {
      return new IptPtr(new trajectory_processing::IterativeParabolicTimeParameterization());
    }
  catch (std::runtime_error& e)
    {
      ROS_ERROR("Exception: [%s]", e.what());
      return 0;
    }
}

MOVIMP(void, IterativeParabolicTimeParameterization, delete)(IptPtr *ptr)
{
  delete ptr;
}

MOVIMP(void, IterativeParabolicTimeParameterization, release)(IptPtr *ptr)
{
  ptr->reset();
}

MOVIMP(bool,IterativeParabolicTimeParameterization , computeTimeStamps)(IptPtr *ptr, RobotTrajectoryPtr *rt)
{
  return (*ptr)->computeTimeStamps (**rt);
}
