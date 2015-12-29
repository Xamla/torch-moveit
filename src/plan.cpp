#include "torch-moveit.h"
#include "utils.h"

MOVIMP(PlanPtr*, Plan, new)()
{
   return new PlanPtr(new moveit::planning_interface::MoveGroup::Plan());
}

MOVIMP(void, Plan, delete)(PlanPtr *ptr)
{
  if (ptr)
    delete ptr;
}

MOVIMP(void, Plan, release)(PlanPtr *ptr)
{
  ptr->reset();
}

MOVIMP(double, Plan, getPlanningTime)(PlanPtr *ptr)
{
  return (*ptr)->planning_time_;
}
