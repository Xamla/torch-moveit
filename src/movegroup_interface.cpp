#include "torch-moveit.h"
#include "utils.h"

MOVIMP(MoveGroupInterfacePtr*, MoveGroupInterface, new)(const char *name, double timeout)
{
  try
    {
      ros::WallDuration timeout_duration = ros::WallDuration();
      if (timeout > 0)
        timeout_duration = ros::WallDuration(timeout);
      return new MoveGroupInterfacePtr(new moveit::planning_interface::MoveGroupInterface(name, boost::shared_ptr<tf::Transformer>(), timeout_duration));
    }
  catch (std::runtime_error& e)
    {
      ROS_ERROR("Exception: [%s]", e.what());
      return 0;
    }
}

MOVIMP(void, MoveGroupInterface, delete)(MoveGroupInterfacePtr *ptr)
{
  delete ptr;
}

MOVIMP(void, MoveGroupInterface, release)(MoveGroupInterfacePtr *ptr)
{
  ptr->reset ();
}

MOVIMP(const char*, MoveGroupInterface, getName)(MoveGroupInterfacePtr *self)
{
  return (*self)->getName().c_str();
}

MOVIMP(const char*, MoveGroupInterface, getPlanningFrame)(MoveGroupInterfacePtr *self)
{
  return (*self)->getPlanningFrame().c_str();
}

MOVIMP(const char*, MoveGroupInterface, getEndEffectorLink)(MoveGroupInterfacePtr *self)
{
  return (*self)->getEndEffectorLink().c_str();
}

MOVIMP(bool, MoveGroupInterface, setJointValueTarget)(MoveGroupInterfacePtr *self, THDoubleTensor *t)
{
  std::vector<double> group_variable_values;
  Tensor2vector(t,group_variable_values);
  return (*self)->setJointValueTarget (group_variable_values);
}

MOVIMP(void, MoveGroupInterface, getJointValueTarget)(MoveGroupInterfacePtr *self, RobotStatePtr *ptr)
{
  ptr->reset(new moveit::core::RobotState((*self)->getJointValueTarget()));
}

MOVIMP(void, MoveGroupInterface, getActiveJoints)(MoveGroupInterfacePtr *self, StringVector *output)
{
  *output = (*self)->getActiveJoints ();
}

MOVIMP(void, MoveGroupInterface, getJoints)(MoveGroupInterfacePtr *self, StringVector *output)
{
  *output = (*self)->getJoints ();
}

MOVIMP(void, MoveGroupInterface, getJointNames)(MoveGroupInterfacePtr *self, StringVector *output)
{
  *output = (*self)->getJointNames();
}

MOVIMP(void, MoveGroupInterface, getLinkNames)(MoveGroupInterfacePtr *self, StringVector *output)
{
  *output = (*self)->getLinkNames();
}

/*MOVIMP(const char*, MoveGroupInterface, getDefaultPlannerId)(MoveGroupInterfacePtr *self, const char* group)
{
  if (group == NULL)
  group = "";
  return (*self)->getDefaultPlannerId(group).c_str();
}*/

MOVIMP(void, MoveGroupInterface, setGoalTolerance)(MoveGroupInterfacePtr *self, double tolerance)
{
  (*self)->setGoalTolerance (tolerance);
}

MOVIMP(double, MoveGroupInterface, getGoalJointTolerance)(MoveGroupInterfacePtr *self)
{
  return (*self)->getGoalJointTolerance ();
}

MOVIMP(void, MoveGroupInterface, setGoalJointTolerance)(MoveGroupInterfacePtr *self, double tolerance)
{
  (*self)->setGoalJointTolerance (tolerance);
}

MOVIMP(double, MoveGroupInterface, getGoalOrientationTolerance)(MoveGroupInterfacePtr *self)
{
  return (*self)->getGoalOrientationTolerance ();
}

MOVIMP(void, MoveGroupInterface, setGoalOrientationTolerance)(MoveGroupInterfacePtr *self, double tolerance)
{
  (*self)->setGoalOrientationTolerance (tolerance);
}

MOVIMP(double, MoveGroupInterface, getGoalPositionTolerance)(MoveGroupInterfacePtr *self)
{
  return (*self)->getGoalPositionTolerance ();
}

MOVIMP(void, MoveGroupInterface, setGoalPositionTolerance)(MoveGroupInterfacePtr *self, double tolerance)
{
  (*self)->setGoalPositionTolerance (tolerance);
}

MOVIMP(void, MoveGroupInterface, setMaxVelocityScalingFactor)(MoveGroupInterfacePtr *self, double factor)
{
  (*self)->setMaxVelocityScalingFactor (factor);
}

MOVIMP(void, MoveGroupInterface, setPlannerId)(MoveGroupInterfacePtr *self, const char *planner_id)
{
  (*self)->setPlannerId (planner_id);
}

MOVIMP(double, MoveGroupInterface, getPlannigTime)(MoveGroupInterfacePtr *self)
{
  return (*self)->getPlanningTime ();
}

MOVIMP(void, MoveGroupInterface, setPlanningTime)(MoveGroupInterfacePtr *self, double seconds)
{
  (*self)->setPlanningTime (seconds);
}

MOVIMP(int, MoveGroupInterface, getVariableCount)(MoveGroupInterfacePtr *self)
{
  return static_cast<int> ((*self)->getVariableCount ());
}

MOVIMP(void, MoveGroupInterface, setNumPlanningAttempts)(MoveGroupInterfacePtr *self, unsigned int attempts)
{
  (*self)->setNumPlanningAttempts (attempts);
}

MOVIMP(void, MoveGroupInterface, setStartStateToCurrentState)(MoveGroupInterfacePtr *self)
{
  (*self)->setStartStateToCurrentState ();
}

MOVIMP(void, MoveGroupInterface, setStartState)(MoveGroupInterfacePtr *self, RobotStatePtr *robot_state)
{
  (*self)->setStartState (**robot_state);
}

MOVIMP(void, MoveGroupInterface, setSupportSurfaceName)(MoveGroupInterfacePtr *self, const char *name)
{
  (*self)->setSupportSurfaceName (name);
}

MOVIMP(void, MoveGroupInterface, setWorkspace)(
    MoveGroupInterfacePtr *self,
    double minx,
    double miny,
    double minz,
    double maxx,
    double maxy,
    double maxz)
{
  (*self)->setWorkspace (minx, miny, minz, maxx, maxy, maxz);
}

MOVIMP(void, MoveGroupInterface, allowLooking)(MoveGroupInterfacePtr *self, bool flag)
{
  (*self)->allowLooking (flag);
}

MOVIMP(void, MoveGroupInterface, allowReplanning)(MoveGroupInterfacePtr* self, bool flag)
{
  (*self)->allowReplanning (flag);
}

MOVIMP(void, MoveGroupInterface, setRandomTarget)(MoveGroupInterfacePtr* self)
{
  (*self)->setRandomTarget ();
}

MOVIMP(bool, MoveGroupInterface, setNamedTarget)(MoveGroupInterfacePtr *self, const char *name)
{
  return (*self)->setNamedTarget (name);
}

MOVIMP(bool, MoveGroupInterface, setPositionTarget_Tensor)(
    MoveGroupInterfacePtr *self,
    THDoubleTensor *t,
    const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  Eigen::Vector3d v = Tensor2Vec3d (t);
  return (*self)->setPositionTarget (v[0], v[1], v[2], end_effector_link);
}

MOVIMP(bool, MoveGroupInterface, setPositionTarget)(
    MoveGroupInterfacePtr *self,
    double x,
    double y,
    double z,
    const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  return (*self)->setPositionTarget (x, y, z, end_effector_link);
}

MOVIMP(bool, MoveGroupInterface, setOrientationTarget_Tensor)(MoveGroupInterfacePtr *self, THDoubleTensor *t, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  Eigen::Vector4d v = Tensor2Vec4d (t);
  return (*self)->setOrientationTarget (v[0], v[1], v[2], v[3],end_effector_link);
}

MOVIMP(bool, MoveGroupInterface, setOrientationTarget)(
    MoveGroupInterfacePtr *self,
    double x,
    double y,
    double z,
    double w,
    const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  return (*self)->setOrientationTarget (x, y, z, w, end_effector_link);
}

MOVIMP(bool, MoveGroupInterface, setRPYTarget)(
    MoveGroupInterfacePtr *self,
    double roll,
    double pitch,
    double yaw,
    const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  return (*self)->setRPYTarget (roll, pitch, yaw, end_effector_link);
}

MOVIMP(bool, MoveGroupInterface, setPoseTarget_Tensor)(MoveGroupInterfacePtr *self, THDoubleTensor *mat, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";

  Eigen::Affine3d t (Tensor2Mat<4, 4> (mat));
  return (*self)->setPoseTarget (t, end_effector_link);
}

MOVIMP(bool, MoveGroupInterface, setPoseTarget_Pose)(MoveGroupInterfacePtr *self, tf::Transform *target, const char *end_effector_link)
{
  geometry_msgs::Pose pose_msg;
  tf::poseTFToMsg(*target, pose_msg);
  if (!end_effector_link)
    end_effector_link = "";
  return (*self)->setPoseTarget(pose_msg, end_effector_link);
}

MOVIMP(void, MoveGroupInterface, setPoseReferenceFrame)(MoveGroupInterfacePtr *self, const char *reference_frame)
{
  (*self)->setPoseReferenceFrame (reference_frame);
}

MOVIMP(void, MoveGroupInterface, setEndEffectorLink)(MoveGroupInterfacePtr *self, const char *name)
{
  (*self)->setEndEffectorLink (name);
}

MOVIMP(const char*, MoveGroupInterface, getEndEffectorlink)(MoveGroupInterfacePtr *self)
  {
    return (*self)->getEndEffectorLink().c_str();
  }

MOVIMP(void, MoveGroupInterface, clearPoseTarget)(MoveGroupInterfacePtr *self, const char *end_effector_link)
{
  if (!end_effector_link)
    end_effector_link = "";
  (*self)->clearPoseTarget (end_effector_link);
}

MOVIMP(void, MoveGroupInterface, clearPoseTargets)(MoveGroupInterfacePtr *self)
{
  (*self)->clearPoseTargets ();
}

MOVIMP(int, MoveGroupInterface, asyncMove)(MoveGroupInterfacePtr *self)
{
  return (*self)->asyncMove ().val;
}

MOVIMP(int, MoveGroupInterface, move)(MoveGroupInterfacePtr *self)
{
  return (*self)->move ().val;
}

MOVIMP(int, MoveGroupInterface, plan)(MoveGroupInterfacePtr *self, PlanPtr *result)
{
  if (!result || !*result)
    throw MoveItWrapperException ("Valid plan output argument has to be provided by caller.");

  moveit::planning_interface::MoveItErrorCode error = (*self)->plan (**result);
  return error.val;
}

MOVIMP(int, MoveGroupInterface, asyncExecute)(MoveGroupInterfacePtr *self, PlanPtr *plan)
{
  moveit::planning_interface::MoveItErrorCode error = (*self)->asyncExecute (**plan);
  return error.val;
}

MOVIMP(int, MoveGroupInterface, execute)(MoveGroupInterfacePtr *self, PlanPtr *plan)
{
  moveit::planning_interface::MoveItErrorCode error = (*self)->execute (**plan);
  return error.val;
}

MOVIMP(void, MoveGroupInterface, setJointPostureConstraint)(
    MoveGroupInterfacePtr *self,
    const char *joint_name,
    double position,
    double tolerance_above,
    double tolerance_below,
    double weight)
{

  // Constructing the joint constraint
  moveit_msgs::JointConstraint jcm;
  jcm.joint_name = joint_name;
  jcm.position = position;
  jcm.tolerance_above = tolerance_above; //1.5;
  jcm.tolerance_below = tolerance_below; //1.5;
  jcm.weight = weight;
  moveit_msgs::Constraints test_constraints;
  test_constraints.joint_constraints.push_back (jcm);
  (*self)->setPathConstraints (test_constraints);
}

MOVIMP(void, MoveGroupInterface, setOrientationConstraint)(
    MoveGroupInterfacePtr *self,
    const char *link_name,
    const char *frame_id,
    double orientation_w,
    double absolute_x_axis_tolerance,
    double absolute_y_axis_tolerance,
    double absolute_z_axis_tolerance,
    double weight)
{
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = link_name; //"r_wrist_roll_link";
  ocm.header.frame_id = frame_id; //"base_link";
  ocm.orientation.w = orientation_w; //1.0;
  ocm.absolute_x_axis_tolerance = absolute_x_axis_tolerance; //0.1;
  ocm.absolute_y_axis_tolerance = absolute_y_axis_tolerance; //;0.1;
  ocm.absolute_z_axis_tolerance = absolute_z_axis_tolerance; //0.1;
  ocm.weight = weight;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back (ocm);
  (*self)->setPathConstraints (test_constraints);
}

MOVIMP(void, MoveGroupInterface, clearPathConstraints)(MoveGroupInterfacePtr *self)
{
  (*self)->clearPathConstraints ();
}

MOVIMP(double, MoveGroupInterface, computeCartesianPath_Tensor)(
    MoveGroupInterfacePtr *self,
    THDoubleTensor *positions,
    THDoubleTensor *orientations,
    double eef_step,
    double jump_threshold,
    bool avoid_collisions,
    int *error_code,
    PlanPtr *plan)
{
  // fill waypoint vecetor from tensors
  std::vector<geometry_msgs::Pose> waypoints;
  ROS_INFO("position size: %d, %d", (int )THDoubleTensor_size (positions, 0),
           (int )THDoubleTensor_size (positions, 1));
  // validate tensor dimensionality
  if (!positions || THDoubleTensor_nDimension (positions) != 2
      || THDoubleTensor_size (positions, 1) != 3)
    throw MoveItWrapperException (
        "A position tensor with 2 dimensions was expected (each row specifying a (x,y,z) 3D position vector).");
  ROS_INFO("orientations size: %d, %d",
           (int )THDoubleTensor_size (orientations, 0),
           (int )THDoubleTensor_size (orientations, 1));
  if (!orientations || THDoubleTensor_nDimension (orientations) != 2
      || THDoubleTensor_size (orientations, 1) != 4)
    throw MoveItWrapperException (
        "A orientation tensor with 2 dimensions was expected (each row specifying a (x,y,z,w) quaternion).");
  int n = THDoubleTensor_size (positions, 0);
  ROS_INFO("orientations size: %d, position size %d",
           (int )THDoubleTensor_size (orientations, 0),
           (int )THDoubleTensor_size (positions, 0));
  if (THDoubleTensor_size (orientations, 0) < n)
    throw MoveItWrapperException (
        "For each position there must be an orientation.");

  // get continous data segments
  positions = THDoubleTensor_newContiguous (positions);
  orientations = THDoubleTensor_newContiguous (orientations);

  double* pos = THDoubleTensor_data (positions);
  double* rot = THDoubleTensor_data (orientations);
  for (int i = 0; i < n; ++i)
    {
      geometry_msgs::Pose p;
      p.position.x = pos[i * 3 + 0];
      p.position.y = pos[i * 3 + 1];
      p.position.z = pos[i * 3 + 2];
      p.orientation.x = rot[i * 4 + 0];
      p.orientation.y = rot[i * 4 + 1];
      p.orientation.z = rot[i * 4 + 2];
      p.orientation.w = rot[i * 4 + 3];
      waypoints.push_back (p);
    }

  THDoubleTensor_free (positions);
  THDoubleTensor_free (orientations);

  moveit_msgs::RobotTrajectory path_msg;
  moveit_msgs::MoveItErrorCodes error;
  double fraction = (*self)->computeCartesianPath (waypoints, eef_step,
                                                   jump_threshold, path_msg,
                                                   avoid_collisions, &error);
  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
           fraction * 100.0);

  robot_trajectory::RobotTrajectory rt (
      (*self)->getCurrentState ()->getRobotModel (), (*self)->getName ());

  // Second get a RobotTrajectory from trajectory
  //rt.setRobotTrajectoryMsg(*((*self)->getCurrentState()), path_msg);

  // Thrid create a IterativeParabolicTimeParameterization object
  //trajectory_processing::IterativeParabolicTimeParameterization iptp;
  //bool success = iptp.computeTimeStamps(rt);

  //ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  // Get RobotTrajectory_msg from RobotTrajectory
  //rt.getRobotTrajectoryMsg(path_msg);

  // Finally plan and execute the trajectory
  (*plan)->trajectory_ = path_msg;

  if (error_code)
    *error_code = error.val;

  return fraction;
}

MOVIMP(bool, MoveGroupInterface, attachObject)(MoveGroupInterfacePtr *self, const char *object, const char *link)
{
  if (!link)
    link = "";
  return (*self)->attachObject (object, link);
}

MOVIMP(bool, MoveGroupInterface, detachObject)(MoveGroupInterfacePtr *self, const char *object)
{
  return (*self)->detachObject (object);
}

MOVIMP(void, MoveGroupInterface, stop)(MoveGroupInterfacePtr *self)
{
  (*self)->stop ();
}

MOVIMP(bool, MoveGroupInterface, startStateMonitor)(MoveGroupInterfacePtr *self, double wait_seconds = 1.0)
{
  return (*self)->startStateMonitor (wait_seconds);
}

MOVIMP(RobotStatePtr *, MoveGroupInterface, getCurrentState)(MoveGroupInterfacePtr *self)
  {
    RobotStatePtr p = (*self)->getCurrentState();
    return new RobotStatePtr(p);
  }

MOVIMP(void, MoveGroupInterface, getCurrentPose_Tensor)(MoveGroupInterfacePtr *self, const char *end_effector_link, THDoubleTensor *output)
{
  moveit::planning_interface::MoveGroupInterface& mg = **self;
  std::string end_effector_link_;
  if (end_effector_link)
    end_effector_link_ = end_effector_link;

  const std::string &eef =
      end_effector_link_.empty () ?
          mg.getEndEffectorLink () : end_effector_link_;
  if (eef.empty ())
    throw MoveItWrapperException ("No end-effector specified.");

  Eigen::Affine3d pose;
  pose.setIdentity ();

  RobotStatePtr current_state = mg.getCurrentState ();
  if (current_state)
    {
      const robot_model::LinkModel *lm = current_state->getLinkModel (eef);
      if (lm)
        pose = current_state->getGlobalLinkTransform (lm);
    }

  copyMatrix (pose.matrix (), output);
}

MOVIMP(void, MoveGroupInterface, getCurrentPose_StampedTransform)(MoveGroupInterfacePtr *self, const char *end_effector_link, tf::StampedTransform *pose)
{
  if (!end_effector_link)
    end_effector_link = "";
  geometry_msgs::PoseStamped msg_pose = (*self)->getCurrentPose (
      end_effector_link);

  tf::Stamped<tf::Pose> stamped_pose;
  tf::poseStampedMsgToTF (msg_pose, stamped_pose);

  static_cast<tf::Transform&> (*pose) =
      static_cast<const tf::Transform&> (stamped_pose);
  pose->stamp_ = stamped_pose.stamp_;
  pose->frame_id_ = stamped_pose.frame_id_;
  pose->child_frame_id_.clear ();
}

MOVIMP(void, MoveGroupInterface, getCurrentPose)(MoveGroupInterfacePtr *self, const char *end_effector_link, tf::Transform *pose)
{
  if (!end_effector_link)
    end_effector_link = "";
  geometry_msgs::PoseStamped msg_pose = (*self)->getCurrentPose (
      end_effector_link);

  tf::Stamped<tf::Pose> stamped_pose;
  tf::poseStampedMsgToTF (msg_pose, stamped_pose);

  *pose = static_cast<const tf::Transform&> (stamped_pose);
}

MOVIMP(int, MoveGroupInterface, pick)(MoveGroupInterfacePtr *self, const char *object)
{
  std::string object_name;
  if (object)
    object_name = object;

  return (*self)->pick (object_name).val;
}

MOVIMP(int, MoveGroupInterface, place)(MoveGroupInterfacePtr *self, const char *object, const tf::Transform *target)
{
  std::vector< geometry_msgs::Pose >  poses;
  geometry_msgs::Pose pose_msg;
  tf::poseTFToMsg(*target, pose_msg);
  poses.push_back(pose_msg);
  std::string object_name;
  if (object)
    object_name = object;

  return (*self)->place (object_name).val;
}

MOVIMP(int, MoveGroupInterface, planGraspsAndPick)(MoveGroupInterfacePtr *self, const char *object)
{
  std::string object_name;
  if (object)
    object_name = object;

  return (*self)->planGraspsAndPick (object_name).val;
}

MOVIMP(void, MoveGroupInterface, rememberJointValues)(MoveGroupInterfacePtr *self, 	const char *name	)
{
  (*self)->rememberJointValues (name);
}
