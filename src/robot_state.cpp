#include "torch-moveit.h"
#include <moveit/robot_state/conversions.h>
#include "utils.h"

MOVIMP(RobotStatePtr *, RobotState, createEmpty)()
{
  return new RobotStatePtr();
}

MOVIMP(RobotStatePtr *, RobotState, createFromModel)(RobotModelPtr *kinematic_model)
{
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(*kinematic_model));
  kinematic_state->setToDefaultValues();
  return new RobotStatePtr(kinematic_state);
}

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

MOVIMP(void, RobotState, setVariableVelocities)(RobotStatePtr *self, THDoubleTensor *view)
{
  std::vector<double> group_variable_values;
  Tensor2vector(view,group_variable_values);
  return (*self)->setVariableVelocities (group_variable_values);
}

MOVIMP(void, RobotState, setVariableVelocities_)(RobotStatePtr *self, THDoubleTensor *t, StringVector *input) {
  std::vector<double> group_variable_values;
  Tensor2vector(t,group_variable_values);
  (*self)->setVariableVelocities(*input, group_variable_values);
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

MOVIMP(void, RobotState, setVariableAccelerations)(RobotStatePtr *self, THDoubleTensor *view)
{
  std::vector<double> group_variable_values;
  Tensor2vector(view,group_variable_values);
  return (*self)->setVariableAccelerations (group_variable_values);
}

MOVIMP(void, RobotState, setVariableAccelerations_)(RobotStatePtr *self, THDoubleTensor *t, StringVector *input) {
  std::vector<double> group_variable_values;
  Tensor2vector(t,group_variable_values);
  (*self)->setVariableAccelerations(*input, group_variable_values);
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

MOVIMP(void, RobotState, setVariableEffort)(RobotStatePtr *self, THDoubleTensor *view)
{
  std::vector<double> group_variable_values;
  Tensor2vector(view,group_variable_values);
  return (*self)->setVariableEffort (group_variable_values);
}

MOVIMP(void, RobotState, setVariableEffort_)(RobotStatePtr *self, THDoubleTensor *t, StringVector *input) {
  std::vector<double> group_variable_values;
  Tensor2vector(t,group_variable_values);
  (*self)->setVariableEffort(*input, group_variable_values);
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
  const char *tip,
  unsigned int attempts,
  double timeout,
  bool return_approximate_solution,
  THDoubleTensor *result_joint_positions
) {
  const robot_state::JointModelGroup *group = (*self)->getJointModelGroup(group_id);

  std::vector<double> joint_values;
  Eigen::Affine3d pose_;
  tf::poseTFToEigen(*pose, pose_);

  kinematics::KinematicsQueryOptions query_options;
  if (return_approximate_solution) {
    query_options.return_approximate_solution = true;
  }
  std::string tip_link = std::string(tip);
  bool found_ik = false;
  if (tip_link.size() == 0 )
  {
    found_ik = (*self)->setFromIK(group, pose_, attempts, timeout, moveit::core::GroupStateValidityCallbackFn(), query_options);
  } else {
    found_ik = (*self)->setFromIK(group, pose_, tip_link, attempts, timeout, moveit::core::GroupStateValidityCallbackFn(), query_options);
  }


  if (found_ik) {
    (*self)->copyJointGroupPositions(group, joint_values);
    vector2Tensor(joint_values, result_joint_positions);
  } else {
    ROS_DEBUG("Did not find IK solution");
  }

  return found_ik;
}

MOVIMP(bool, RobotState, getGlobalLinkTransform)(
  RobotStatePtr *self,
  tf::Transform *pose_,
  const char *link_name
) {
  const moveit::core::LinkModel* model = (*self)->getLinkModel(link_name);
  if (model == NULL)
  {
    return false;
  }
  Eigen::Affine3d pose = (*self)->getGlobalLinkTransform(model);
  tf::poseEigenToTF(pose, *pose_);
  return true;
}

MOVIMP(void, RobotState, setVariablePositions)(RobotStatePtr *self, THDoubleTensor *t) {
  std::vector<double> group_variable_values;
  Tensor2vector(t,group_variable_values);
  (*self)->setVariablePositions(group_variable_values);
}

MOVIMP(void, RobotState, setVariablePositions_)(RobotStatePtr *self, THDoubleTensor *t, StringVector *input) {
  std::vector<double> group_variable_values;
  Tensor2vector(t,group_variable_values);
  (*self)->setVariablePositions(*input, group_variable_values);
}

MOVIMP(void, RobotState, updateLinkTransforms)(RobotStatePtr *self) {
  (*self)->updateLinkTransforms();
}

MOVIMP(void, RobotState, update)(RobotStatePtr *self) {
  (*self)->update();
}

MOVIMP(void, RobotState, toRobotStateMsg)(RobotStatePtr *self, THByteStorage *output, bool copy_attached_bodies) {
  moveit_msgs::RobotState robot_state;

  robot_state::robotStateToRobotStateMsg(**self, robot_state, copy_attached_bodies);

  uint32_t length = ros::serialization::serializationLength(robot_state);
  THByteStorage_resize(output, length + sizeof(uint32_t));
  ros::serialization::OStream stream(THByteStorage_data(output), length + sizeof(uint32_t));
  stream.next((uint32_t)length);
  ros::serialization::serialize(stream, robot_state);
}

MOVIMP(void, RobotState, fromRobotStateMsg)(RobotStatePtr *self, THByteStorage *serialized_msg)
{
  long storage_size = THByteStorage_size(serialized_msg);

  uint8_t *data = THByteStorage_data(serialized_msg);

  ros::serialization::IStream stream(data+ sizeof(uint32_t), static_cast<uint32_t>(storage_size-sizeof(uint32_t)));
  moveit_msgs::RobotState rs_msg;
  ros::serialization::Serializer<moveit_msgs::RobotState>::read(stream, rs_msg);
  robot_state::robotStateMsgToRobotState(rs_msg, (**self));
}

MOVIMP(void, RobotState, getJointTransform)(RobotStatePtr *self, const char *joint_name, THDoubleTensor *result) {
  const Eigen::Affine3d & pose = (*self)->getJointTransform (joint_name);
  copyMatrix(pose.matrix(), result);
}

MOVIMP(void, RobotState, getJacobian)(RobotStatePtr *self, const char *group_id, THDoubleTensor *result) {
  const robot_state::JointModelGroup *group = (*self)->getJointModelGroup(group_id);
  Eigen::Vector3d linkOrigin = Eigen::Vector3d::Zero();
  Eigen::MatrixXd jacobian; //= (*self)->getJacobian (group,linkOrigin);
  (*self)->getJacobian(group, (*self)->getLinkModel(group->getLinkModelNames().back()),linkOrigin, jacobian);
  THDoubleTensor_resize2d(result, jacobian.rows(), jacobian.cols());
  THDoubleTensor* result_ = THDoubleTensor_newContiguous(result);
  Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> m (THDoubleTensor_data(result_),jacobian.rows(), jacobian.cols());
  m = jacobian;
  THDoubleTensor_freeCopyTo(result_, result);
}

MOVIMP(void, RobotState, enforceBounds)(RobotStatePtr *self) {
  (*self)->enforceBounds();
}

MOVIMP(double, RobotState, distance)(RobotStatePtr *self, RobotStatePtr *other) {
  return (*self)->distance(**other);
}

MOVIMP(bool, RobotState, satisfiesBounds)(RobotStatePtr *self, double margin) {
  return (*self)->satisfiesBounds(margin);
}

MOVIMP(void, RobotState, copyJointGroupPositions)(RobotStatePtr *self, const char *group_id, THDoubleTensor *result) {
  std::vector<double> joint_group_positions;
  (*self)->copyJointGroupPositions(group_id, joint_group_positions);
  vector2Tensor(joint_group_positions,result);
}
/*
void 	printDirtyInfo (std::ostream &out=std::cout) const
void 	printStateInfo (std::ostream &out=std::cout) const
void 	printStatePositions (std::ostream &out=std::cout) const
void 	printTransform (const Eigen::Affine3d &transform, std::ostream &out=std::cout) const
void 	printTransforms (std::ostream &out=std::cout) const
*/
