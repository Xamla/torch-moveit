local ffi = require 'ffi'

local moveit = {}

moveit.std = {}
moveit.ros = {}
moveit.tf = {}

-- std
local std_cdef = [[
typedef struct std_string {} std_string;
typedef struct std_StringVector {} std_StringVector;

std_string* std_string_new(const char *s, size_t len);
void std_string_delete(std_string *self);
std_string* std_string_clone(std_string *self);
void std_string_assign(std_string *self, const char *s, size_t len);
int std_string_length(std_string *self);
const char* std_string_c_str(std_string *self);

std_StringVector *std_StringVector_new();
std_StringVector *std_StringVector_clone(std_StringVector *self);
void std_StringVector_delete(std_StringVector *ptr);
int std_StringVector_size(std_StringVector *self);
const char* std_StringVector_getAt(std_StringVector *self, size_t pos);
void std_StringVector_setAt(std_StringVector *self, size_t pos, const char *value);
void std_StringVector_push_back(std_StringVector *self, const char *value);
void std_StringVector_pop_back(std_StringVector *self);
void std_StringVector_clear(std_StringVector *self);
void std_StringVector_insert(std_StringVector *self, size_t pos, size_t n, const char *value);
void std_StringVector_erase(std_StringVector *self, size_t begin, size_t end);
bool std_StringVector_empty(std_StringVector *self);
]]

ffi.cdef(std_cdef)

-- ros
local ros_cdef = [[
typedef struct ros_AsyncSpinner {} ros_AsyncSpinner;
typedef struct ros_Time {} ros_Time;
typedef struct ros_Duration {} ros_Duration;

void ros___init(const char *name, uint32_t options);
void ros___shutdown();
void ros___spinOnce();
void ros___requestShutdown();
bool ros___isInitialized();
bool ros___isStarted();
bool ros___isShuttingDown();
bool ros___ok();
void ros___waitForShutdown();

ros_AsyncSpinner* ros_AsyncSpinner_new(uint32_t thread_count);
void ros_AsyncSpinner_delete(ros_AsyncSpinner *self);
bool ros_AsyncSpinner_canStart(ros_AsyncSpinner *self);
void ros_AsyncSpinner_start(ros_AsyncSpinner *self);
void ros_AsyncSpinner_stop(ros_AsyncSpinner *self);

ros_Time* ros_Time_new();
void ros_Time_delete(ros_Time *self);
ros_Time* ros_Time_clone(ros_Time *self);
bool ros_Time_isZero(ros_Time *self);
void ros_Time_fromSec(ros_Time *self, double t);
double ros_Time_toSec(ros_Time *self);
void ros_Time_set(ros_Time *self, unsigned int sec, unsigned int nsec);
void ros_Time_assign(ros_Time *self, ros_Time *other);
int ros_Time_get_sec(ros_Time *self);
void ros_Time_set_sec(ros_Time *self, unsigned int sec);
int ros_Time_get_nsec(ros_Time *self);
void ros_Time_set_nesc(ros_Time *self, unsigned int nsec);
bool ros_Time_lt(ros_Time *self, ros_Time *other);
bool ros_Time_eq(ros_Time *self, ros_Time *other);
void ros_Time_add_Duration(ros_Time *self, ros_Duration *duration, ros_Time *result);
void ros_Time_sub(ros_Time *self, ros_Time *other, ros_Duration *result);
void ros_Time_sub_Duration(ros_Time *self, ros_Duration *duration, ros_Time *result);
void ros_Time_sleepUntil(ros_Time *end);
void ros_Time_getNow(ros_Time *result);
void ros_Time_setNow(ros_Time* now);
void ros_Time_waitForValid();
void ros_Time_init();
void ros_Time_shutdown();
bool ros_Time_useSystemTime();
bool ros_Time_isSimTime();
bool ros_Time_isSystemTime();
bool ros_Time_isValid();

ros_Duration* ros_Duration_new();
void ros_Duration_delete(ros_Duration *self);
ros_Duration* ros_Duration_clone(ros_Duration *self);
void ros_Duration_set(ros_Duration *self, int sec, int nsec);
void ros_Duration_assign(ros_Duration *self, ros_Duration *other);
int ros_Duration_get_sec(ros_Duration *self);
void ros_Duration_set_sec(ros_Duration *self, int sec);
int ros_Duration_get_nsec(ros_Duration *self);
void ros_Duration_set_nsec(ros_Duration *self, int nsec);
void ros_Duration_add(ros_Duration *self, ros_Duration *other, ros_Duration *result);
void ros_Duration_sub(ros_Duration *self, ros_Duration *other, ros_Duration *result);
void ros_Duration_mul(ros_Duration *self, double scale, ros_Duration *result);
bool ros_Duration_eq(ros_Duration *self, ros_Duration *other);
bool ros_Duration_lt(ros_Duration *self, ros_Duration *other);
double ros_Duration_toSec(ros_Duration *self);
void ros_Duration_fromSec(ros_Duration *self, double t);
bool ros_Duration_isZero(ros_Duration *self);
void ros_Duration_sleep(ros_Duration *self);
]]

ffi.cdef(ros_cdef)

-- tf
local tf_cdef = [[
typedef struct tf_Transform {} tf_Transform;
typedef struct tf_StampedTransform {} tf_StampedTransform;
typedef struct tf_Quaternion {} tf_Quaternion;
typedef struct tf_TransformBroadcaster {} tf_TransformBroadcaster;
typedef struct tf_TransformListener {} tf_TransformListener;

tf_Quaternion * tf_Quaternion_new();
tf_Quaternion * tf_Quaternion_clone(tf_Quaternion *self);
void tf_Quaternion_delete(tf_Quaternion *self);
void tf_Quaternion_setIdentity(tf_Quaternion *self);
void tf_Quaternion_setRotation_Tensor(tf_Quaternion *self, THDoubleTensor *axis, double angle);
void tf_Quaternion_setEuler(tf_Quaternion *self, double yaw, double pitch, double roll);
void tf_Quaternion_setRPY(tf_Quaternion *self, double roll, double pitch, double yaw);
void tf_Quaternion_setEulerZYX(tf_Quaternion *self, double yaw, double pitch, double roll);
double tf_Quaternion_getAngle(tf_Quaternion *self);
void tf_Quaternion_getAxis_Tensor(tf_Quaternion *self, THDoubleTensor *axis);
void tf_Quaternion_inverse(tf_Quaternion *self, tf_Quaternion *result);
double tf_Quaternion_length2(tf_Quaternion *self);
void tf_Quaternion_normalize(tf_Quaternion *self);
double tf_Quaternion_angle(tf_Quaternion *self, tf_Quaternion *other);
double tf_Quaternion_angleShortestPath(tf_Quaternion *self, tf_Quaternion *other);
void tf_Quaternion_add(tf_Quaternion *self, tf_Quaternion *other, tf_Quaternion *result);
void tf_Quaternion_sub(tf_Quaternion *self, tf_Quaternion *other, tf_Quaternion *result);
void tf_Quaternion_mul(tf_Quaternion *self, tf_Quaternion *other, tf_Quaternion *result);
void tf_Quaternion_mul_scalar(tf_Quaternion *self, double factor, tf_Quaternion *result);
void tf_Quaternion_div_scalar(tf_Quaternion *self, double divisor, tf_Quaternion *result);
double tf_Quaternion_dot(tf_Quaternion *self, tf_Quaternion *other);
void tf_Quaternion_slerp(tf_Quaternion *self, tf_Quaternion *other, double t, tf_Quaternion *result);
void tf_Quaternion_viewTensor(tf_Quaternion *self, THDoubleTensor* result);

tf_Transform *tf_Transform_new();
tf_Transform *tf_Transform_clone(tf_Transform *self);
void tf_Transform_delete(tf_Transform *self);
void tf_Transform_setIdentity(tf_Transform *self);
void tf_Transform_mul_Quaternion(tf_Transform *self, tf_Quaternion *rot, tf_Quaternion *result);
void tf_Transform_mul_Transform(tf_Transform *self, tf_Transform *other, tf_Transform *result);
void tf_Transform_inverse(tf_Transform *self, tf_Transform *result);
void tf_Transform_getBasis(tf_Transform *self, THDoubleTensor *basis);
void tf_Transform_getOrigin(tf_Transform *self, THDoubleTensor *origin);
void tf_Transform_setRotation(tf_Transform *self, tf_Quaternion *rotation);
void tf_Transform_getRotation(tf_Transform *self, tf_Quaternion *rotation);

tf_StampedTransform *tf_StampedTransform_new(tf_Transform *transform, ros_Time* timestamp, const char *frame_id, const char *child_frame_id);
tf_StampedTransform *tf_StampedTransform_clone(tf_StampedTransform *self);
void tf_StampedTransform_delete(tf_StampedTransform *self);
tf_Transform *tf_StampedTransform_getBasePointer(tf_StampedTransform *self);
void tf_StampedTransform_get_stamp(tf_StampedTransform *self, ros_Time *result);
void tf_StampedTransform_set_stamp(tf_StampedTransform *self, ros_Time *stamp);
const char *tf_StampedTransform_get_frame_id(tf_StampedTransform *self);
void tf_StampedTransform_set_frame_id(tf_StampedTransform *self, const char *id);
const char *tf_StampedTransform_get_child_frame_id(tf_StampedTransform *self);
void tf_StampedTransform_set_child_frame_id(tf_StampedTransform *self, const char *id);
void tf_StampedTransform_setData(tf_StampedTransform *self, tf_Transform *input);
bool tf_StampedTransform_eq(tf_StampedTransform *self, tf_StampedTransform *other);

tf_TransformBroadcaster *tf_TransformBroadcaster_new();
void tf_TransformBroadcaster_delete(tf_TransformBroadcaster *self);
void tf_TransformBroadcaster_sendTransform(tf_TransformBroadcaster *self, tf_StampedTransform *transform);

tf_TransformListener * tf_TransformListener_new();
void tf_TransformListener_delete(tf_TransformListener *self);
void tf_TransformListener_clear(tf_TransformListener *self);
void tf_TransformListener_getFrameStrings(tf_TransformListener *self, std_StringVector *result);
void tf_TransformListener_lookupTransform(tf_TransformListener *self, const char *target_frame, const char *source_frame, ros_Time *time, tf_StampedTransform *result);
bool tf_TransformListener_waitForTransform(tf_TransformListener *self, const char *target_frame, const char *source_frame, ros_Time *time, ros_Duration *timeout, std_string *error_msg);
bool tf_TransformListener_canTransform(tf_TransformListener *self, const char *target_frame, const char *source_frame, ros_Time *time, std_string *error_msg);
void tf_TransformListener_resolve(tf_TransformListener *self, const char *frame_name, std_string *result);
]]

ffi.cdef(tf_cdef)

-- movit
local moveit_cdef = [[
typedef struct Point { double x; double y; double z; } Point;
typedef struct Quaternion { double x; double y; double z; double w; } Quaternion;
typedef struct Pose { Point position; Quaternion orientation; } Pose;

typedef struct MoveItModulePtr {} MoveItModulePtr;
typedef struct MoveGroupPtr {} MoveGroupPtr;
typedef struct PlanPtr {} PlanPtr;
typedef struct RobotStatePtr {} RobotStatePtr;

MoveItModulePtr *moveit_TorchMoveItModule_new();
void moveit_TorchMoveItModule_delete(MoveItModulePtr *ptr);

MoveGroupPtr* moveit_MoveGroup_new(const char *name);
void moveit_MoveGroup_delete(MoveGroupPtr *ptr);
void moveit_MoveGroup_release(MoveGroupPtr *ptr);
const char* moveit_MoveGroup_getName(MoveGroupPtr *self);
const char* moveit_MoveGroup_getPlanningFrame(MoveGroupPtr *self);
const char* moveit_MoveGroup_getEndEffectorLink(MoveGroupPtr *self);
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
bool moveit_MoveGroup_setOrientationTarget(MoveGroupPtr *self, double x, double y, double z, double w, const char *end_effector_link);
bool moveit_MoveGroup_setOrientationTarget_Tensor(MoveGroupPtr *self, THDoubleTensor *t, const char *end_effector_link);
bool moveit_MoveGroup_setRPYTarget(MoveGroupPtr *self, double roll, double pitch, double yaw, const char *end_effector_link);
bool moveit_MoveGroup_setPoseTarget_Transform(MoveGroupPtr *self, THDoubleTensor *mat, const char *end_effector_link);
bool moveit_MoveGroup_setPoseTarget_Pose(MoveGroupPtr *self, const Pose &target, const char *end_effector_link);
void moveit_MoveGroup_setPoseReferenceFrame(MoveGroupPtr *self, const char *reference_frame);
void moveit_MoveGroup_clearPoseTarget(MoveGroupPtr *self, const char *end_effector_link);
void moveit_MoveGroup_clearPoseTargets(MoveGroupPtr *self);
int moveit_MoveGroup_asyncMove(MoveGroupPtr *self);
int moveit_MoveGroup_move(MoveGroupPtr *self);
int moveit_MoveGroup_plan(MoveGroupPtr *self, PlanPtr *plan_output);
int moveit_MoveGroup_asyncExecute(MoveGroupPtr *self, PlanPtr *plan);
int moveit_MoveGroup_execute(MoveGroupPtr *self, PlanPtr *plan);
double moveit_MoveGroup_computeCartesianPath_Tensor(MoveGroupPtr *self, THDoubleTensor *positions, THDoubleTensor *orientations, double eef_step, double jump_threshold, bool avoid_collisions, int *error_code);
bool moveit_MoveGroup_attachObject(MoveGroupPtr *self, const char *object, const char *link);
bool moveit_MoveGroup_detachObject(MoveGroupPtr *self, const char *object);
void moveit_MoveGroup_stop(MoveGroupPtr *self);
bool moveit_MoveGroup_startStateMonitor(MoveGroupPtr *self, double wait);
RobotStatePtr *moveit_MoveGroup_getCurrentState(MoveGroupPtr *self);
void moveit_MoveGroup_getCurrentPose_Transform(MoveGroupPtr *self, const char *end_effector_link, THDoubleTensor* output);
void moveit_MoveGroup_getCurrentPose(MoveGroupPtr *self, const char *end_effector_link, Pose *pose);
void moveit_MoveGroup_getCurrentPose_Tensors(MoveGroupPtr *self, const char *end_effector_link, THDoubleTensor *position, THDoubleTensor *orientation);

PlanPtr* moveit_Plan_new();
void moveit_Plan_delete(PlanPtr *ptr);
void moveit_Plan_release(PlanPtr *ptr);
double moveit_Plan_getPlanningTime(PlanPtr *ptr);

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

void moveit_Pose_getRotation(THDoubleTensor* m, THDoubleTensor* quaternion_out);
void moveit_Pose_setRotation(THDoubleTensor* m, THDoubleTensor* quaternion_in);
]]

ffi.cdef(moveit_cdef)

moveit.lib = ffi.load(package.searchpath('libmoveit', package.cpath))

return moveit
