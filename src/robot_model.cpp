#include "torch-moveit.h"
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include "utils.h"


MOVIMP(RobotModelPtr*, RobotModel, new)()
{
   return new RobotModelPtr();
}

MOVIMP(void, RobotModel, delete)(RobotModelPtr *ptr)
{
  if (ptr)
    delete ptr;
}

MOVIMP(void, RobotModel, release)(RobotModelPtr *ptr)
{
  ptr->reset();
}

MOVIMP(const char *, RobotModel, getName)(RobotModelPtr *ptr)
{
  return (*ptr)->getName().c_str();
}

MOVIMP(const char *, RobotModel, getModelFrame)(RobotModelPtr *ptr)
{
  return (*ptr)->getModelFrame().c_str();
}

MOVIMP(bool, RobotModel, isEmpty)(RobotModelPtr *ptr)
{
  return (*ptr)->isEmpty();
}

MOVIMP(void, RobotModel, printModelInfo)(RobotModelPtr *ptr, std::string *output)
{
  std::stringstream s;
  (*ptr)->printModelInfo(s);
  *output = s.str();
}

MOVIMP(const char *, RobotModel, getRootJointName)(RobotModelPtr *ptr)
{
  return (*ptr)->getRootJointName().c_str();
}

MOVIMP(void, RobotModel, getEndEffectorNames)(RobotModelPtr *ptr, StringVector *output)
{
  const std::vector<moveit::core::JointModelGroup*> jmg = (*ptr)->getJointModelGroups();
  for (std::vector<moveit::core::JointModelGroup*>::const_iterator it = jmg.begin(); it != jmg.end(); it++)
    output->push_back((*it)->getEndEffectorName());
}

MOVIMP(bool, RobotModel, getGroupEndEffectorName)(RobotModelPtr *ptr,  const char *groupname, std::string *output)
{
  const moveit::core::JointModelGroup* jmgp = (*ptr)->getJointModelGroup(groupname);
  if (jmgp)
  {
    *output = jmgp->getEndEffectorName();
    return true;
  }
  return false;
}

MOVIMP(bool, RobotModel, getEndEffectorLinkName)(RobotModelPtr *ptr,  const char *eef_name, std::string *output)
{
  const robot_model::JointModelGroup* jmg = (*ptr)->getEndEffector(eef_name);
  if (jmg)
  {
    *output = jmg->getEndEffectorParentGroup().second;
    return true;
  }
  return false;
}

MOVIMP(void, RobotModel, getGroupEndEffectorNames)(RobotModelPtr *self, const char *name, StringVector *output)
{
  //*output = (*self)->getJointModelGroup(name)->getAttachedEndEffectorNames ();
  (*self)->getJointModelGroup(name)->getEndEffectorTips(*output);
}

MOVIMP(void, RobotModel, getJointModelGroupNames)(RobotModelPtr *ptr, StringVector *output)
{
  const StringVector jmg = (*ptr)->getJointModelGroupNames();
  for (StringVector::const_iterator it = jmg.begin(); it != jmg.end(); it++)
    output->push_back((*it));
}

MOVIMP(void, RobotModel, getJointModelSubGroupNames)(RobotModelPtr *ptr, const char *groupname, StringVector *output)
{
  const moveit::core::JointModelGroup* jmgp = (*ptr)->getJointModelGroup(groupname);
  std::vector<const moveit::core::JointModelGroup*> jmg;
  jmgp->getSubgroups(jmg);
  for (std::vector<const moveit::core::JointModelGroup*>::const_iterator it = jmg.begin(); it != jmg.end(); it++)
    output->push_back((*it)->getName());
}

MOVIMP(void, RobotModel, getEndEffectorParentGroups)(RobotModelPtr *ptr, StringVector *output1, StringVector *output2)
{
  const std::vector<moveit::core::JointModelGroup*> jmg = (*ptr)->getJointModelGroups();
  for (std::vector<moveit::core::JointModelGroup*>::const_iterator it = jmg.begin(); it != jmg.end(); it++){
    const std::pair< std::string,std::string > info = (*it)->getEndEffectorParentGroup();
    output1->push_back(info.first); // move group name
    output2->push_back(info.second);// link name
  }
}

MOVIMP(void, RobotModel, getJointModelNames)(RobotModelPtr *self, StringVector *output)
{
  *output = (*self)->getJointModelNames();
}

MOVIMP(void, RobotModel, getVariableNames)(RobotModelPtr *self, StringVector *output)
{
  *output = (*self)->getVariableNames();
}

MOVIMP(int, RobotModel, getVariableIndex)(RobotModelPtr *self, const char *name)
{
  return (*self)->getVariableIndex(name);
}

MOVIMP(void, RobotModel, getGroupJointNames)(RobotModelPtr *self, const char *name, StringVector *output)
{
  *output = (*self)->getJointModelGroup(name)->getActiveJointModelNames();
}

MOVIMP(void, RobotModel, getVariableBounds)
(RobotModelPtr *self, THDoubleTensor *limits_position,
  THDoubleTensor *limits_velocity,
  THDoubleTensor *limits_acceleration)
{
  std::vector<std::pair<double, double>> position_limits;
  std::vector<std::pair<double, double>> velocity_limits;
  std::vector<std::pair<double, double>> acceleration_limits;

  StringVector names = (*self)->getVariableNames();
  for (StringVector::const_iterator it = names.begin(); it != names.end();++it){
    moveit::core::VariableBounds bounds = (*self)->getVariableBounds(*it);
    std::pair<double, double> pos_limit;
    std::pair<double, double> vel_limit;
    std::pair<double, double> acc_limit;

    if (bounds.position_bounded_)
    {
      pos_limit.first = bounds.max_position_;
      pos_limit.second = bounds.min_position_;
    }
    else
    {
      pos_limit.first = std::numeric_limits<double>::infinity();
      pos_limit.second = -std::numeric_limits<double>::infinity();
    }

    if (bounds.velocity_bounded_)
    {
      vel_limit.first = bounds.max_velocity_;
      vel_limit.second = bounds.min_velocity_;
    }
    else
    {
      pos_limit.first = std::numeric_limits<double>::infinity();
      pos_limit.second = -std::numeric_limits<double>::infinity();
    }

    if (bounds.acceleration_bounded_)
    {
      acc_limit.first = bounds.max_acceleration_;
      acc_limit.second = bounds.min_acceleration_;
    }
    else
    {
      acc_limit.first = std::numeric_limits<double>::infinity();
      acc_limit.second = -std::numeric_limits<double>::infinity();
    }
    position_limits.push_back(pos_limit);
    velocity_limits.push_back(vel_limit);
    acceleration_limits.push_back(acc_limit);
  }
  vectorPair2Tensor(position_limits, limits_position);
  vectorPair2Tensor(velocity_limits, limits_velocity);
  vectorPair2Tensor(acceleration_limits, limits_acceleration);
}
