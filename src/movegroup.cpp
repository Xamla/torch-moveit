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

MOVIMP(void, MoveGroup, release)(MoveGroupPtr *ptr)
{
  ptr->reset();
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

MOVIMP(void, MoveGroup, getJoints)(MoveGroupPtr *self, StringVector *output)
{
  *output = (*self)->getJoints();
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

MOVIMP(bool, MoveGroup, setPositionTarget_Tensor)(MoveGroupPtr *self, THDoubleTensor *t, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  Eigen::Vector3d v = Tensor2Vec3d(t);
  return (*self)->setPositionTarget(v[0], v[1], v[2], end_effector_link);
}

MOVIMP(bool, MoveGroup, setPositionTarget)(MoveGroupPtr *self, double x, double y, double z, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  return (*self)->setPositionTarget(x, y, z, end_effector_link);
}

MOVIMP(bool, MoveGroup, setOrientationTarget_Tensor)(MoveGroupPtr *self, THDoubleTensor *t, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  Eigen::Vector4d v = Tensor2Vec4d(t);
  return (*self)->setOrientationTarget(v[0], v[1], v[2], v[3], end_effector_link);
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

MOVIMP(bool, MoveGroup, setPoseTarget_Transform)(MoveGroupPtr *self, THDoubleTensor *mat, const char *end_effector_link)
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

MOVIMP(void, MoveGroup, setPoseReferenceFrame)(MoveGroupPtr *self, const char *reference_frame)
{
  (*self)->setPoseReferenceFrame(reference_frame);
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

MOVIMP(int, MoveGroup, asyncMove)(MoveGroupPtr *self)
{
  return (*self)->asyncMove().val;
}

MOVIMP(int, MoveGroup, move)(MoveGroupPtr *self)
{
  return (*self)->move().val;
}

MOVIMP(int, MoveGroup, plan)(MoveGroupPtr *self, PlanPtr *result)
{
  if (!result || !*result)
    throw MoveItWrapperException("Valid plan output argument has to be provided by caller.");
    
  return (*self)->plan(**result);
}

MOVIMP(int, MoveGroup, asyncExecute)(MoveGroupPtr *self, PlanPtr *plan)
{
  return (*self)->asyncExecute(**plan);
}

MOVIMP(int, MoveGroup, execute)(MoveGroupPtr *self, PlanPtr *plan)
{
  return (*self)->execute(**plan);
}

MOVIMP(double, MoveGroup, computeCartesianPath_Tensor)(MoveGroupPtr *self, THDoubleTensor *positions, THDoubleTensor *orientations, double eef_step, double jump_threshold, bool avoid_collisions, int *error_code)
{
  // fill waypoint vecetor from tensors
  std::vector<geometry_msgs::Pose> waypoints;
  
  // validate tensor dimensionality  
  if (!positions || THDoubleTensor_nDimension(positions) != 2 || THDoubleTensor_size(positions, 1) != 3)
    throw MoveItWrapperException("A position tensor with 2 dimensions was expected (each row specifying a (x,y,z) 3D position vector).");
    
  if (!orientations || THDoubleTensor_nDimension(orientations) != 2 || THDoubleTensor_size(orientations, 1) != 4)
    throw MoveItWrapperException("A orientation tensor with 2 dimensions was expected (each row specifying a (x,y,z,w) quaternion).");

  int n = THDoubleTensor_size(positions, 0);
  if (THDoubleTensor_size(orientations, 0) < n)
    throw MoveItWrapperException("For each position there must be an orientation.");
  
  // get continous data segments
  positions = THDoubleTensor_newContiguous(positions);
  orientations = THDoubleTensor_newContiguous(orientations);
  
  double* pos = THDoubleTensor_data(positions);
  double* rot = THDoubleTensor_data(orientations);

  for (int i = 0; i < n; ++i, rot+=3, rot+=4)
  {
    geometry_msgs::Pose p;
    p.position.x = pos[0];
    p.position.y = pos[1];
    p.position.z = pos[2];
    p.orientation.x = rot[0];
    p.orientation.y = rot[1];
    p.orientation.z = rot[2];
    p.orientation.w = rot[3];
    waypoints.push_back(p);
  }
  
  THDoubleTensor_free(positions);
  THDoubleTensor_free(orientations);
  
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::MoveItErrorCodes error;
  double result = (*self)->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions, &error);
  if (error_code)
    *error_code = error.val;
  return result;
}

MOVIMP(bool, MoveGroup, attachObject)(MoveGroupPtr *self, const char *object, const char *link)
{
  if (!link)
    link = "";
  (*self)->attachObject(object, link);
}

MOVIMP(bool, MoveGroup, detachObject)(MoveGroupPtr *self, const char *object)
{
  (*self)->detachObject(object);
}

MOVIMP(void, MoveGroup, stop)(MoveGroupPtr *self)
{
  (*self)->stop();
}

MOVIMP(bool, MoveGroup, startStateMonitor)(MoveGroupPtr *self, double wait_seconds = 1.0)
{
  return (*self)->startStateMonitor(wait_seconds);
}

MOVIMP(RobotStatePtr *, MoveGroup, getCurrentState)(MoveGroupPtr *self)
{
  robot_state::RobotStatePtr p = (*self)->getCurrentState();
  return new RobotStatePtr(p);
}

MOVIMP(void, MoveGroup, getCurrentPose_Transform)(MoveGroupPtr *self, const char *end_effector_link, THDoubleTensor *output)
{
  moveit::planning_interface::MoveGroup& mg = **self;
  std::string end_effector_link_;  
  if (end_effector_link)
    end_effector_link_ = end_effector_link;
  
  const std::string &eef = end_effector_link_.empty() ? mg.getEndEffectorLink() : end_effector_link_;
  if (eef.empty())
    throw MoveItWrapperException("No end-effector specified.");
    
  Eigen::Affine3d pose;
  pose.setIdentity();
  
  RobotStatePtr current_state = mg.getCurrentState();
  if (current_state)
  {
    const robot_model::LinkModel *lm = current_state->getLinkModel(eef);
    if (lm)
      pose = current_state->getGlobalLinkTransform(lm);
  }
  
  copyMatrix(pose.matrix(), output);
}

MOVIMP(void, MoveGroup, getCurrentPose)(MoveGroupPtr *self, const char *end_effector_link, geometry_msgs::Pose *pose)
{
  if (!end_effector_link)
    end_effector_link = "";
  geometry_msgs::PoseStamped pose_ = (*self)->getCurrentPose(end_effector_link);
  pose->position = pose_.pose.position;
  pose->orientation = pose_.pose.orientation;
}

MOVIMP(void, MoveGroup, getCurrentPose_Tensors)(MoveGroupPtr *self, const char *end_effector_link, THDoubleTensor *position, THDoubleTensor *orientation)
{
  if (!end_effector_link)
    end_effector_link = "";
  geometry_msgs::Pose pose = (*self)->getCurrentPose(end_effector_link).pose;
  Eigen::Vector3d p;
  p[0] = pose.position.x;
  p[1] = pose.position.y;
  p[2] = pose.position.z;
  Eigen::Vector4d o;
  o[0] = pose.orientation.x;
  o[0] = pose.orientation.y;
  o[0] = pose.orientation.z;
  o[0] = pose.orientation.w;
  copyMatrix(p, position);
  copyMatrix(o, orientation);
}
