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

MOVIMP(void, RobotState, getVariableNames)(RobotStatePtr *self, StringsPtr *output)
{
  **output = (*self)->getVariableNames();
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


/*
void 	printDirtyInfo (std::ostream &out=std::cout) const
void 	printStateInfo (std::ostream &out=std::cout) const
void 	printStatePositions (std::ostream &out=std::cout) const
void 	printTransform (const Eigen::Affine3d &transform, std::ostream &out=std::cout) const
void 	printTransforms (std::ostream &out=std::cout) const 
*/
