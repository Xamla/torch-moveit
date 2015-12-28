local ffi = require 'ffi'

local moveit = {}

local cdef = [[

typedef struct Point { double x; double y; double z; } Point;
typedef struct Quaternion { double x; double y; double z; double w; } Quaternion;
typedef struct Pose { Point position; Quaternion orientation; } Pose;

typedef struct MoveItModulePtr {} MoveItModulePtr;
typedef struct MoveGroupPtr {} MoveGroupPtr;
typedef struct PlanPtr {} PlanPtr;

MoveItModulePtr *moveit_TorchMoveItModule_new();
void moveit_TorchMoveItModule_delete(MoveItModulePtr *ptr);

MoveGroupPtr* moveit_MoveGroup_new(const char *name);
void moveit_MoveGroup_delete(MoveGroupPtr *ptr);
const char* moveit_MoveGroup_getName(MoveGroupPtr *self);
const char* moveit_MoveGroup_getPlanningFrame(MoveGroupPtr *self);
const char* moveit_MoveGroup_getEndEffectorLink(MoveGroupPtr *self);
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
bool moveit_MoveGroup_setOrientationTarget(MoveGroupPtr *self, double x, double y, double z, double w, const char *end_effector_link);
bool moveit_MoveGroup_setRPYTarget(MoveGroupPtr *self, double roll, double pitch, double yaw, const char *end_effector_link);
bool moveit_MoveGroup_setPoseTarget_Tensor(MoveGroupPtr *self, THDoubleTensor *mat, const char *end_effector_link);
bool moveit_MoveGroup_setPoseTarget_Pose(MoveGroupPtr *self, const Pose &target, const char *end_effector_link);
void moveit_MoveGroup_setEndEffectorLink(MoveGroupPtr *self, const char *name);
const char* moveit_MoveGroup_getEndEffectorlink(MoveGroupPtr *self);
void moveit_MoveGroup_clearPoseTarget(MoveGroupPtr *self, const char *end_effector_link);
void moveit_MoveGroup_clearPoseTargets(MoveGroupPtr *self);
int moveit_MoveGroup_move(MoveGroupPtr *self);
void moveit_MoveGroup_stop(MoveGroupPtr *self);

PlanPtr* moveit_Plan_new();
void moveit_Plan_delete(PlanPtr *ptr);
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
