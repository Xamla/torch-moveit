#include "torch-moveit.h"
#include "utils.h"

MOVIMP(RobotStatePtr *, RobotState, clone)(RobotStatePtr *ptr)
{
  return new RobotStatePtr(new moveit::core::RobotState(**ptr));
}

MOVIMP(void, RobotState, delete)(RobotStatePtr *ptr)
{
  delete ptr;
}

MOVIMP(void, RobotState, release)(RobotStatePtr *ptr)
{
  ptr->reset();
}

MOVIMP(int, RobotState, getVariableCount)(RobotStatePtr *self)
{
  return static_cast<int>((*self)->getVariableCount());
}

MOVIMP(void, RobotState, getVariableNames)(RobotStatePtr *self, StringVector *output)
{
  *output = (*self)->getVariableNames();
}

MOVIMP(void, RobotState, getVariablePositions)(RobotStatePtr *self, THDoubleTensor *view)
{
  double *data = (*self)->getVariablePositions();
  size_t count = (*self)->getVariableCount();
  viewArray(data, count, view);
}

MOVIMP(bool, RobotState, hasVelocities)(RobotStatePtr *self)
{
  return (*self)->hasVelocities();
}

MOVIMP(void, RobotState, getVariableVelocities)(RobotStatePtr *self, THDoubleTensor *view)
{
  double *data = (*self)->getVariableVelocities();
  size_t count = (*self)->getVariableCount();
  viewArray(data, count, view);
}

MOVIMP(bool, RobotState, hasAccelerations)(RobotStatePtr *self)
{
  return (*self)->hasAccelerations();
}

MOVIMP(void, RobotState, getVariableAccelerations)(RobotStatePtr *self, THDoubleTensor *view)
{
  double *data = (*self)->getVariableAccelerations();
  size_t count = (*self)->getVariableCount();
  viewArray(data, count, view);
}

MOVIMP(bool, RobotState, hasEffort)(RobotStatePtr *self)
{
  return (*self)->hasEffort();
}

MOVIMP(void, RobotState, getVariableEffort)(RobotStatePtr *self, THDoubleTensor *view)
{
  double *data = (*self)->getVariableEffort();
  size_t count = (*self)->getVariableCount();
  viewArray(data, count, view);
}

MOVIMP(void, RobotState, setToDefaultValues)(RobotStatePtr *self)
{
  (*self)->setToDefaultValues();
}

MOVIMP(void, RobotState, setToRandomPositions)(RobotStatePtr *self)
{
  (*self)->setToRandomPositions();
}

MOVIMP(bool, RobotState, setFromIK)
(RobotStatePtr *self, const char * group_id_, const tf::Transform * pose_,
 THDoubleTensor * tensor)
{

  std::string group_id = std::string (group_id_);
  const robot_state::JointModelGroup* group = (*self)->getJointModelGroup (
      group_id);
  Eigen::Affine3d pose;
  tf::poseTFToEigen (*pose_, pose);

  std::vector<double> joint_values;

  const std::vector<std::string> &joint_names = group->getJointModelNames ();
  bool found_ik = (*self)->setFromIK (group, pose, 10, 0.1);
  if (found_ik)
    {

      (*self)->copyJointGroupPositions (group, joint_values);
      THDoubleTensor_resize1d (tensor, joint_values.size ());
      THDoubleTensor * output = THDoubleTensor_newContiguous (tensor);
      double *data = THDoubleTensor_data (output);

      for (std::size_t i = 0; i < joint_values.size (); ++i)
        {
          ROS_DEBUG("Joint %d: %f", (int)i, joint_values[i]);
          data[i] = joint_values[i];
        }
      THDoubleTensor_freeCopyTo (output, tensor);
    }
  else
    {
      ROS_INFO("Did not find IK solution");
    }
  return found_ik;
}


/*
void 	printDirtyInfo (std::ostream &out=std::cout) const
void 	printStateInfo (std::ostream &out=std::cout) const
void 	printStatePositions (std::ostream &out=std::cout) const
void 	printTransform (const Eigen::Affine3d &transform, std::ostream &out=std::cout) const
void 	printTransforms (std::ostream &out=std::cout) const 
*/
