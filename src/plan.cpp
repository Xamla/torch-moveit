#include "torch-moveit.h"
#include "utils.h"

MOVIMP(PlanPtr*, Plan, new)()
{
   return new PlanPtr(new moveit::planning_interface::MoveGroup::Plan());
}

MOVIMP(void, Plan, delete)(PlanPtr *ptr)
{
  delete ptr;
}
