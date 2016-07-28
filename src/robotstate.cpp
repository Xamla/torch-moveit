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

MOVIMP(bool, RobotState, setFromIK)(
  RobotStatePtr *self,
  const char *group_id,
  const tf::Transform *pose,
  unsigned int attempts,
  double timeout,
  THDoubleTensor *result_joint_positions
) {
  const robot_state::JointModelGroup *group = (*self)->getJointModelGroup(group_id);

  std::vector<double> joint_values;
  Eigen::Affine3d pose_;
  tf::poseTFToEigen(*pose, pose_);
  bool found_ik = (*self)->setFromIK(group, pose_, attempts, timeout);
  if (found_ik) {
    (*self)->copyJointGroupPositions(group, joint_values);
    vector2Tensor(joint_values, result_joint_positions);
  } else {
    ROS_DEBUG("Did not find IK solution");
  }

  return found_ik;
}

MOVIMP(void, RobotState, getGlobalLinkTransform)(
  RobotStatePtr *self,
  tf::Transform *pose_,
  const char *link_name
) {
  Eigen::Affine3d pose = (*self)->getGlobalLinkTransform(link_name);
  tf::poseEigenToTF(pose,*pose_);
}

MOVIMP(void, RobotState, setVariablePositions)(RobotStatePtr *self, THDoubleTensor *t) {
  std::vector<double> group_variable_values;
  Tensor2vector(t,group_variable_values);
  (*self)->setVariablePositions(group_variable_values);
}

MOVIMP(void, RobotState, updateLinkTransforms)(RobotStatePtr *self) {
  (*self)->updateLinkTransforms();
}

/*
void 	printDirtyInfo (std::ostream &out=std::cout) const
void 	printStateInfo (std::ostream &out=std::cout) const
void 	printStatePositions (std::ostream &out=std::cout) const
void 	printTransform (const Eigen::Affine3d &transform, std::ostream &out=std::cout) const
void 	printTransforms (std::ostream &out=std::cout) const
*/
