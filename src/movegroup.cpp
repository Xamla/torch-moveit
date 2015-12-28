#include "torch-moveit.h"
#include "utils.h"

MOVIMP(MoveGroupPtr*, MoveGroup, new)(const char *name)
{
   return new MoveGroupPtr(new moveit::planning_interface::MoveGroup(name));
}

MOVIMP(void, MoveGroup, delete)(MoveGroupPtr *ptr)
{
  delete ptr;
}

MOVIMP(const char*, MoveGroup, getName)(MoveGroupPtr *self)
{
  return (*self)->getName().c_str();
}

MOVIMP(const char*, MoveGroup, getPlanningFrame)(MoveGroupPtr *self)
{
  return (*self)->getPlanningFrame().c_str();
}

MOVIMP(const char*, MoveGroup, getEndEffectorLink)(MoveGroupPtr *self)
{
  return (*self)->getEndEffectorLink().c_str();
}

/*MOVIMP(const char*, MoveGroup, getDefaultPlannerId)(MoveGroupPtr *self, const char* group)
{
  if (group == NULL)
    group = "";
  return (*self)->getDefaultPlannerId(group).c_str();
}*/

MOVIMP(void, MoveGroup, setGoalTolerance)(MoveGroupPtr *self, double tolerance)
{
  (*self)->setGoalTolerance(tolerance);
}

MOVIMP(double, MoveGroup, getGoalJointTolerance)(MoveGroupPtr *self)
{
  return (*self)->getGoalJointTolerance();
}

MOVIMP(void, MoveGroup, setGoalJointTolerance)(MoveGroupPtr *self, double tolerance)
{
  (*self)->setGoalJointTolerance(tolerance);
}

MOVIMP(double, MoveGroup, getGoalOrientationTolerance)(MoveGroupPtr *self) 
{
  return (*self)->getGoalOrientationTolerance();
}

MOVIMP(void, MoveGroup, setGoalOrientationTolerance)(MoveGroupPtr *self, double tolerance)
{
  (*self)->setGoalOrientationTolerance(tolerance);
}

MOVIMP(double, MoveGroup, getGoalPositionTolerance)(MoveGroupPtr *self)
{
  return (*self)->getGoalPositionTolerance();
}

MOVIMP(void, MoveGroup, setGoalPositionTolerance)(MoveGroupPtr *self, double tolerance)
{
  (*self)->setGoalPositionTolerance(tolerance);
}

MOVIMP(void, MoveGroup, setMaxVelocityScalingFactor)(MoveGroupPtr *self, double factor)
{
  (*self)->setMaxVelocityScalingFactor(factor);
}

MOVIMP(void, MoveGroup, setPlannerId)(MoveGroupPtr *self, const char *planner_id)
{
  (*self)->setPlannerId(planner_id);
}

MOVIMP(double, MoveGroup, getPlannigTime)(MoveGroupPtr *self)
{
  return (*self)->getPlanningTime();
}

MOVIMP(void, MoveGroup, setPlanningTime)(MoveGroupPtr *self, double seconds)
{
  (*self)->setPlanningTime(seconds);
}

MOVIMP(int, MoveGroup, getVariableCount)(MoveGroupPtr *self)
{
  return static_cast<int>((*self)->getVariableCount());
}

MOVIMP(void, MoveGroup, setNumPlanningAttempts)(MoveGroupPtr *self, unsigned int attempts)
{
  (*self)->setNumPlanningAttempts(attempts);
}

MOVIMP(void, MoveGroup, setStartStateToCurrentState)(MoveGroupPtr *self)
{
  (*self)->setStartStateToCurrentState();
}
 
MOVIMP(void, MoveGroup, setSupportSurfaceName)(MoveGroupPtr *self, const char *name)
{
  (*self)->setSupportSurfaceName(name);
}

MOVIMP(void, MoveGroup, setWorkspace)(MoveGroupPtr *self, double minx, double miny, double minz, double maxx, double maxy, double maxz)
{
  (*self)->setWorkspace(minx, miny, minz, maxx, maxy, maxz);
}

MOVIMP(void, MoveGroup, allowLooking)(MoveGroupPtr *self, bool flag)
{
  (*self)->allowLooking(flag);
}

MOVIMP(void, MoveGroup, allowReplanning)(MoveGroupPtr* self, bool flag)
{
  (*self)->allowReplanning(flag);
}

MOVIMP(void, MoveGroup, setRandomTarget)(MoveGroupPtr* self)
{
  (*self)->setRandomTarget();
}

MOVIMP(bool, MoveGroup, setNamedTarget)(MoveGroupPtr *self, const char *name)
{
  return (*self)->setNamedTarget(name);
}

MOVIMP(bool, MoveGroup, setPositionTarget)(MoveGroupPtr *self, double x, double y, double z, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  return (*self)->setPositionTarget(x, y, z, end_effector_link);
}

MOVIMP(bool, MoveGroup, setOrientationTarget)(MoveGroupPtr *self, double x, double y, double z, double w, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  return (*self)->setOrientationTarget(x, y, z, w, end_effector_link);
}

MOVIMP(bool, MoveGroup, setRPYTarget)(MoveGroupPtr *self, double roll, double pitch, double yaw, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  return (*self)->setRPYTarget(roll, pitch, yaw, end_effector_link);
}

MOVIMP(bool, MoveGroup, setPoseTarget_Tensor)(MoveGroupPtr *self, THDoubleTensor *mat, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
    
  Eigen::Affine3d t(Tensor2Mat<4,4>(mat));
  return (*self)->setPoseTarget(t, end_effector_link);
}

MOVIMP(bool, MoveGroup, setPoseTarget_Pose)(MoveGroupPtr *self, const geometry_msgs::Pose &target, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  return (*self)->setPoseTarget(target);
}

/*MOVIMP(bool, MoveGroup, setPoseTarget_PoseStamped)(MoveGroupPtr *self, const geometry_msgs::PoseStamped &target, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  return (*self)->setPoseTarget(target);
}*/

MOVIMP(void, MoveGroup, setEndEffectorLink)(MoveGroupPtr *self, const char *name)
{
  (*self)->setEndEffectorLink(name);
}

MOVIMP(const char*, MoveGroup, getEndEffectorlink)(MoveGroupPtr *self)
{
  return (*self)->getEndEffectorLink().c_str();
}

MOVIMP(void, MoveGroup, clearPoseTarget)(MoveGroupPtr *self, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  (*self)->clearPoseTarget(end_effector_link);
}
  
MOVIMP(void, MoveGroup, clearPoseTargets)(MoveGroupPtr *self)
{
  (*self)->clearPoseTargets();
}

MOVIMP(int, MoveGroup, move)(MoveGroupPtr *self)
{
  return (*self)->move().val;
}

MOVIMP(void, MoveGroup, stop)(MoveGroupPtr *self)
{
  (*self)->stop();
}
