local ffi = require 'ffi'

local moveit = {}

local cdef = [[

typedef struct Point { double x; double y; double z; } Point;
typedef struct Quaternion { double x; double y; double z; double w; } Quaternion;
typedef struct Pose { Point position; Quaternion orientation; } Pose;

typedef struct MoveItModulePtr {} MoveItModulePtr;
typedef struct MoveGroupPtr {} MoveGroupPtr;
typedef struct PlanPtr {} PlanPtr;
typedef struct RobotStatePtr {} RobotStatePtr;
typedef struct StringsPtr {} StringsPtr;

MoveItModulePtr *moveit_TorchMoveItModule_new();
void moveit_TorchMoveItModule_delete(MoveItModulePtr *ptr);

MoveGroupPtr* moveit_MoveGroup_new(const char *name);
void moveit_MoveGroup_delete(MoveGroupPtr *ptr);
void moveit_MoveGroup_release(MoveGroupPtr *ptr);
const char* moveit_MoveGroup_getName(MoveGroupPtr *self);
const char* moveit_MoveGroup_getPlanningFrame(MoveGroupPtr *self);
const char* moveit_MoveGroup_getEndEffectorLink(MoveGroupPtr *self);
void moveit_MoveGroup_setEndEffectorLink(MoveGroupPtr *self, const char *name);
void moveit_MoveGroup_getJoints(MoveGroupPtr *self, StringsPtr *strings);
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

StringsPtr *moveit_Strings_new();
StringsPtr *moveit_Strings_clone(StringsPtr *self);
void moveit_Strings_delete(StringsPtr *ptr);
int moveit_Strings_size(StringsPtr *self);
const char* moveit_Strings_getAt(StringsPtr *self, size_t pos);
void moveit_Strings_setAt(StringsPtr *self, size_t pos, const char *value);
void moveit_Strings_push_back(StringsPtr *self, const char *value);
void moveit_Strings_pop_back(StringsPtr *self);
void moveit_Strings_clear(StringsPtr *self);
void moveit_Strings_insert(StringsPtr *self, size_t pos, size_t n, const char *value);
void moveit_Strings_erase(StringsPtr *self, size_t begin, size_t end);
bool moveit_Strings_empty(StringsPtr *self);

RobotStatePtr *moveit_RobotState_clone(RobotStatePtr *ptr);
void moveit_RobotState_delete(RobotStatePtr *ptr);
void moveit_RobotState_release(RobotStatePtr *ptr);
int moveit_RobotState_getVariableCount(RobotStatePtr *self);
void moveit_RobotState_getVariableNames(RobotStatePtr *self, StringsPtr *output);
void moveit_RobotState_getVariablePositions(RobotStatePtr *self, THDoubleTensor *view);
bool moveit_RobotState_hasVelocities(RobotStatePtr *self);
void moveit_RobotState_getVariableVelocities(RobotStatePtr *self, THDoubleTensor *view);
bool moveit_RobotState_hasAccelerations(RobotStatePtr *self);
void moveit_RobotState_getVariableAccelerations(RobotStatePtr *self, THDoubleTensor *view);
bool moveit_RobotState_hasEffort(RobotStatePtr *self);
void moveit_RobotState_getVariableEffort(RobotStatePtr *self, THDoubleTensor *view);
void moveit_RobotState_setToDefaultValues(RobotStatePtr *self);
void moveit_RobotState_setToRandomPositions(RobotStatePtr *self);
]];

ffi.cdef(cdef)

moveit.lib = ffi.load(package.searchpath('libmoveit', package.cpath))

function moveit.init()
  moveit.module = moveit.lib.moveit_TorchMoveItModule_new()
end

function moveit.shutdown()
  if moveit.module then
    moveit.lib.moveit_TorchMoveItModule_delete(moveit.module);
  end
end

return moveit
