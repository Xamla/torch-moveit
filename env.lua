local ffi = require 'ffi'

local moveit = {}

-- moveit
local moveit_cdef = [[
typedef struct MoveItModulePtr {} MoveItModulePtr;
typedef struct MoveGroupPtr {} MoveGroupPtr;
typedef struct PlanPtr {} PlanPtr;
typedef struct RobotStatePtr {} RobotStatePtr;
typedef struct IptPtr {} IptPtr;
typedef struct CollisionObjectPtr {} CollisionObjectPtr;
typedef struct PlanningSceneInterfacePtr {} PlanningSceneInterfacePtr;
typedef struct RobotModelLoaderPtr {} RobotModelLoaderPtr;
typedef struct RobotModelPtr {} RobotModelPtr;
typedef struct RobotTrajectoryPtr {} RobotTrajectoryPtr;
typedef struct PlanningScenePtr {} PlanningScenePtr;

MoveItModulePtr *moveit_TorchMoveItModule_new();
void moveit_TorchMoveItModule_delete(MoveItModulePtr *ptr);

MoveGroupPtr *moveit_MoveGroup_new(const char *name);
void moveit_MoveGroup_delete(MoveGroupPtr *ptr);
void moveit_MoveGroup_release(MoveGroupPtr *ptr);
const char *moveit_MoveGroup_getName(MoveGroupPtr *self);
const char *moveit_MoveGroup_getPlanningFrame(MoveGroupPtr *self);
const char *moveit_MoveGroup_getEndEffectorLink(MoveGroupPtr *self);
void moveit_MoveGroup_setEndEffectorLink(MoveGroupPtr *self, const char *name);
void moveit_MoveGroup_getJoints(MoveGroupPtr *self, std_StringVector *strings);
void moveit_MoveGroup_setGoalTolerance(MoveGroupPtr *self, double tolerance);
double moveit_MoveGroup_getGoalJointTolerance(MoveGroupPtr *self);
void moveit_MoveGroup_setGoalJointTolerance(MoveGroupPtr *self, double tolerance);
double moveit_MoveGroup_getGoalOrientationTolerance(MoveGroupPtr *self);
void moveit_MoveGroup_setGoalOrientationTolerance(MoveGroupPtr *self, double tolerance);
double moveit_MoveGroup_getGoalPositionTolerance(MoveGroupPtr *self);
void moveit_MoveGroup_setGoalPositionTolerance(MoveGroupPtr *self, double tolerance);
void moveit_MoveGroup_setMaxVelocityScalingFactor(MoveGroupPtr *self, double factor);
void moveit_MoveGroup_setPlannerId(MoveGroupPtr *self, const char *planner_id);
double moveit_MoveGroup_getPlannigTime(MoveGroupPtr *self);
void moveit_MoveGroup_setPlanningTime(MoveGroupPtr *self, double seconds);
int moveit_MoveGroup_getVariableCount(MoveGroupPtr *self);
void moveit_MoveGroup_setNumPlanningAttempts(MoveGroupPtr *self, unsigned int attempts);
void moveit_MoveGroup_setStartStateToCurrentState(MoveGroupPtr *self);
void moveit_MoveGroup_setSupportSurfaceName(MoveGroupPtr *self, const char *name);
void moveit_MoveGroup_setWorkspace(MoveGroupPtr *self, double minx, double miny, double minz, double maxx, double maxy, double maxz);
void moveit_MoveGroup_allowLooking(MoveGroupPtr *self, bool flag);
void moveit_MoveGroup_allowReplanning(MoveGroupPtr* self, bool flag);
void moveit_MoveGroup_setRandomTarget(MoveGroupPtr* self);
bool moveit_MoveGroup_setNamedTarget(MoveGroupPtr *self, const char *name);
bool moveit_MoveGroup_setPositionTarget(MoveGroupPtr *self, double x, double y, double z, const char *end_effector_link);
bool moveit_MoveGroup_setPositionTarget_Tensor(MoveGroupPtr *self, THDoubleTensor *t, const char *end_effector_link);
void moveit_MoveGroup_setJointPostureConstraint(MoveGroupPtr *self, const char *joint_name, double position,double tolerance_above,double tolerance_below , double weight);
bool moveit_MoveGroup_setOrientationTarget(MoveGroupPtr *self, double x, double y, double z, double w, const char *end_effector_link);
bool moveit_MoveGroup_setOrientationTarget_Tensor(MoveGroupPtr *self, THDoubleTensor *t, const char *end_effector_link);
bool moveit_MoveGroup_setRPYTarget(MoveGroupPtr *self, double roll, double pitch, double yaw, const char *end_effector_link);
bool moveit_MoveGroup_setPoseTarget_Tensor(MoveGroupPtr *self, THDoubleTensor *mat, const char *end_effector_link);
bool moveit_MoveGroup_setPoseTarget_Pose(MoveGroupPtr *self, tf_Transform *target, const char *end_effector_link);
void moveit_MoveGroup_setPoseReferenceFrame(MoveGroupPtr *self, const char *reference_frame);
bool moveit_MoveGroup_setJointValueTarget(MoveGroupPtr *self, THDoubleTensor *t);
void moveit_MoveGroup_getJointValueTarget(MoveGroupPtr *self, RobotStatePtr *ptr);
void moveit_MoveGroup_clearPoseTarget(MoveGroupPtr *self, const char *end_effector_link);
void moveit_MoveGroup_clearPoseTargets(MoveGroupPtr *self);
int moveit_MoveGroup_asyncMove(MoveGroupPtr *self);
int moveit_MoveGroup_move(MoveGroupPtr *self);
int moveit_MoveGroup_plan(MoveGroupPtr *self, PlanPtr *plan_output);
int moveit_MoveGroup_asyncExecute(MoveGroupPtr *self, PlanPtr *plan);
int moveit_MoveGroup_execute(MoveGroupPtr *self, PlanPtr *plan);
bool moveit_MoveGroup_attachObject(MoveGroupPtr *self, const char *object, const char *link);
bool moveit_MoveGroup_detachObject(MoveGroupPtr *self, const char *object);
void moveit_MoveGroup_stop(MoveGroupPtr *self);
bool moveit_MoveGroup_startStateMonitor(MoveGroupPtr *self, double wait);
void moveit_MoveGroup_setStartState(MoveGroupPtr *self, RobotStatePtr robot_state);
RobotStatePtr *moveit_MoveGroup_getCurrentState(MoveGroupPtr *self);
void moveit_MoveGroup_getCurrentPose_Tensor(MoveGroupPtr *self, const char *end_effector_link, THDoubleTensor* output);
void moveit_MoveGroup_getCurrentPose_StampedTransform(MoveGroupPtr *self, const char *end_effector_link, tf_StampedTransform *pose);
void moveit_MoveGroup_getCurrentPose(MoveGroupPtr *self, const char *end_effector_link, tf_Transform *pose);

void moveit_MoveGroup_setOrientationConstraint(MoveGroupPtr *self, const char *link_name, const char *frame_id, double orientation_w, double absolute_x_axis_tolerance, double absolute_y_axis_tolerance, double absolute_z_axis_tolerance, double weight);
void moveit_MoveGroup_clearPathConstraints(MoveGroupPtr *self);
double moveit_MoveGroup_computeCartesianPath_Tensor(MoveGroupPtr *self, THDoubleTensor *positions, THDoubleTensor *orientations, double eef_step, double jump_threshold, bool avoid_collisions, int *error_code,PlanPtr *plan);
void moveit_MoveGroup_pick(MoveGroupPtr *self, const char *object);
PlanPtr *moveit_Plan_new();
void moveit_Plan_delete(PlanPtr *ptr);
void moveit_Plan_release(PlanPtr *ptr);
void moveit_Plan_getStartStateMsg(PlanPtr *ptr, THByteStorage *output);
void moveit_Plan_getTrajectoryMsg(PlanPtr *ptr, THByteStorage *output);
void moveit_Plan_setTrajectoryMsg(PlanPtr *ptr, THByteStorage *serialized_msg);
void moveit_Plan_setStartStateMsg(PlanPtr *ptr, THByteStorage *serialized_msg);
double moveit_Plan_getPlanningTime(PlanPtr *ptr);

RobotStatePtr *moveit_RobotState_createEmpty();
RobotStatePtr *moveit_RobotState_clone(RobotStatePtr *ptr);
void moveit_RobotState_delete(RobotStatePtr *ptr);
void moveit_RobotState_release(RobotStatePtr *ptr);
int moveit_RobotState_getVariableCount(RobotStatePtr *self);
void moveit_RobotState_getVariableNames(RobotStatePtr *self, std_StringVector *output);
void moveit_RobotState_getVariablePositions(RobotStatePtr *self, THDoubleTensor *view);
bool moveit_RobotState_hasVelocities(RobotStatePtr *self);
void moveit_RobotState_getVariableVelocities(RobotStatePtr *self, THDoubleTensor *view);
bool moveit_RobotState_hasAccelerations(RobotStatePtr *self);
void moveit_RobotState_getVariableAccelerations(RobotStatePtr *self, THDoubleTensor *view);
bool moveit_RobotState_hasEffort(RobotStatePtr *self);
void moveit_RobotState_getVariableEffort(RobotStatePtr *self, THDoubleTensor *view);
void moveit_RobotState_setToDefaultValues(RobotStatePtr *self);
void moveit_RobotState_setToRandomPositions(RobotStatePtr *self);
bool moveit_RobotState_setFromIK(RobotStatePtr *self, const char *group_id, const tf_Transform *pose, unsigned int attempts, double timeout, THDoubleTensor *result_joint_positions);
void moveit_RobotState_getGlobalLinkTransform(RobotStatePtr *self, tf_Transform *pose, const char *link_name);
void moveit_RobotState_setVariablePositions(RobotStatePtr *self, THDoubleTensor *t);
void moveit_RobotState_updateLinkTransforms(RobotStatePtr *self);

void moveit_Pose_getRotation(THDoubleTensor* m, THDoubleTensor* quaternion_out);
void moveit_Pose_setRotation(THDoubleTensor* m, THDoubleTensor* quaternion_in);

CollisionObjectPtr *moveit_CollisionObject_new();
void moveit_CollisionObject_delete(CollisionObjectPtr *ptr);
const char *moveit_CollisionObject_getId(CollisionObjectPtr *self);
void moveit_CollisionObject_setId(CollisionObjectPtr *self, const char *id);
const char *moveit_CollisionObject_getFrameId(CollisionObjectPtr *self);
void moveit_CollisionObject_setFrameId(CollisionObjectPtr *self, const char *id);
int moveit_CollisionObject_getOperation(CollisionObjectPtr *self);
void moveit_CollisionObject_setOperation(CollisionObjectPtr *self, int operation);
void moveit_CollisionObject_addPrimitive(CollisionObjectPtr *self, int type, THDoubleTensor *dimensions, tf_Transform *pose);
void moveit_CollisionObject_addPlane(CollisionObjectPtr *self, THDoubleTensor *coefs, tf_Transform *pose);

PlanningSceneInterfacePtr *moveit_PlanningSceneInterface_new();
void moveit_PlanningSceneInterface_delete(PlanningSceneInterfacePtr *ptr);
void moveit_PlanningSceneInterface_addCollisionObject(PlanningSceneInterfacePtr *self, CollisionObjectPtr *obj);
void moveit_PlanningSceneInterface_removeCollisionObjects(PlanningSceneInterfacePtr *self, std_StringVector *object_ids);
void moveit_PlanningSceneInterface_getKnownObjectNames(PlanningSceneInterfacePtr *self, bool with_type, std_StringVector *result);
void moveit_PlanningSceneInterface_getKnownObjectNamesInROI(PlanningSceneInterfacePtr *self, double minx, double miny, double minz, double maxx, double maxy, double maxz, bool with_type, std_StringVector* types, std_StringVector* result);
void moveit_PlanningSceneInterfacePtr_getObjectPoses(PlanningSceneInterfacePtr *self, std_StringVector *object_ids, std_StringVector *found, THDoubleTensor *found_poses);

RobotModelLoaderPtr *moveit_RobotModelLoader_new(const char *robot_description, bool load_kinematics_solvers);
void moveit_RobotModelLoader_delete(RobotModelLoaderPtr *ptr);
void moveit_RobotModelLoader_release(RobotModelLoaderPtr *ptr);
void moveit_RobotModelLoader_getModel(RobotModelLoaderPtr *ptr, RobotModelPtr *output);
const char *moveit_RobotModelLoader_getRobotDescription(RobotModelLoaderPtr *ptr);

RobotModelPtr *moveit_RobotModel_new();
void moveit_RobotModel_delete(RobotModelPtr *ptr);
void moveit_RobotModel_release(RobotModelPtr *ptr);
const char *moveit_RobotModel_getName(RobotModelPtr *ptr);
const char *moveit_RobotModel_getModelFrame(RobotModelPtr *ptr);
bool moveit_RobotModel_isEmpty(RobotModelPtr *ptr);
void moveit_RobotModel_printModelInfo(RobotModelPtr *ptr, std_string *output);
const char *moveit_RobotModel_getRootJointName(RobotModelPtr *ptr);

RobotTrajectoryPtr *moveit_RobotTrajectory_new(RobotModelPtr* robot_model, const char *group);
void moveit_RobotTrajectory_delete(RobotTrajectoryPtr *ptr);
void moveit_RobotTrajectory_release(RobotTrajectoryPtr *ptr);
const char *moveit_RobotTrajectory_getGroupName(RobotTrajectoryPtr *ptr);
bool moveit_RobotTrajectory_empty(RobotTrajectoryPtr *ptr);
void moveit_RobotTrajectory_setGroupName(RobotTrajectoryPtr *ptr, const char *group_name);
int moveit_RobotTrajectory_getWayPointCount(RobotTrajectoryPtr *ptr);
void moveit_RobotTrajectory_setWayPointDurationFromPrevious(RobotTrajectoryPtr *ptr, int index, double value);
void moveit_RobotTrajectory_addSuffixWayPoint(RobotTrajectoryPtr *ptr,RobotStatePtr *state, double dt);
void moveit_RobotTrajectory_addPrefixWayPoint(RobotTrajectoryPtr *ptr,RobotStatePtr *state, double dt);
void moveit_RobotTrajectory_insertWayPoint(RobotTrajectoryPtr *ptr,int index, RobotStatePtr *state, double dt);
void moveit_RobotTrajectory_append(RobotTrajectoryPtr *ptr, const RobotTrajectoryPtr *source, double dt);
void moveit_RobotTrajectory_swap(RobotTrajectoryPtr *ptr,RobotTrajectoryPtr *other);
void moveit_RobotTrajectory_clear(RobotTrajectoryPtr *ptr);
double moveit_RobotTrajectory_getAverageSegmentDuration(RobotTrajectoryPtr *ptr);
void moveit_RobotTrajectory_reverse(RobotTrajectoryPtr *ptr);
void moveit_RobotTrajectory_unwind(RobotTrajectoryPtr *ptr);
void moveit_RobotTrajectory_findWayPointIndicesForDurationAfterStart(RobotTrajectoryPtr *ptr,const double &duration, int &before, int &after, double &blend);



PlanningScenePtr* moveit_PlanningScene_new(RobotModelPtr *robot_model);
void moveit_PlanningScene_delete(PlanningScenePtr *ptr);
void moveit_PlanningScene_release(PlanningScenePtr *ptr);
void moveit_PlanningScene_setCurrentState(PlanningScenePtr *ptr, RobotStatePtr *robot_state);
bool moveit_PlanningScene_checkSelfCollision(PlanningScenePtr *ptr, RobotStatePtr *robot_state);
bool moveit_PlanningScene_isStateColliding(PlanningScenePtr *ptr, const char group_name, bool verbose);

IptPtr *moveit_IterativeParabolicTimeParameterization_new();
void moveit_IterativeParabolicTimeParameterization_delete(IptPtr *ptr);
void moveit_IterativeParabolicTimeParameterization_release(IptPtr *ptr);
]]

ffi.cdef(moveit_cdef)

moveit.lib = ffi.load(package.searchpath('libmoveit', package.cpath))

return moveit
