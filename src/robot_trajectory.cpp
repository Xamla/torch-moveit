#include "torch-moveit.h"
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "utils.h"

typedef robot_model::RobotModelPtr RobotModelPtr;
typedef robot_trajectory::RobotTrajectoryPtr RobotTrajectoryPtr;

MOVIMP(RobotTrajectoryPtr*, RobotTrajectory, new)(RobotModelPtr* robot_model, const char *group)
{
   return new RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(*robot_model, group));
}

MOVIMP(void, RobotTrajectory, delete)(RobotTrajectoryPtr *ptr)
{
  if (ptr)
    delete ptr;
}

MOVIMP(void, RobotTrajectory, release)(RobotTrajectoryPtr *ptr)
{
  ptr->reset();
}

MOVIMP(const char *, RobotTrajectory, getGroupName)(RobotTrajectoryPtr *ptr)
{
  return (*ptr)->getGroupName().c_str();
}

MOVIMP(bool, RobotTrajectory, empty)(RobotTrajectoryPtr *ptr)
{
  return (*ptr)->empty();
}

MOVIMP(void, RobotTrajectory, setGroupName)(RobotTrajectoryPtr *ptr, const char *group_name)
{
  (*ptr)->setGroupName(group_name);
}

MOVIMP(int, RobotTrajectory,getWayPointCount)(RobotTrajectoryPtr *ptr)
 {
 // std::size_t
  return (*ptr)->getWayPointCount();
 }

MOVIMP(const robot_state::RobotState&, RobotTrajectory, getWayPoint)(RobotTrajectoryPtr *ptr,std::size_t index)
 {
  return (*ptr)->getWayPoint(index);
 }

MOVIMP(const robot_state::RobotState &, RobotTrajectory, getLastWayPoint)(RobotTrajectoryPtr *ptr)
 {
  return (*ptr)->getLastWayPoint();
 }

MOVIMP(const robot_state::RobotState &, RobotTrajectory, getFirstWayPoint)(RobotTrajectoryPtr *ptr)
 {
  return (*ptr)->getFirstWayPoint();
 }

MOVIMP(const robot_state::RobotStatePtr &, RobotTrajectory, getWayPointPtr)(RobotTrajectoryPtr *ptr, std::size_t index)
 {
  return (*ptr)->getWayPointPtr(index);
 }

MOVIMP(robot_state::RobotStatePtr &, RobotTrajectory, getLastWayPointPtr)(RobotTrajectoryPtr *ptr)
 {
  return (*ptr)->getLastWayPointPtr();
 }

MOVIMP(robot_state::RobotStatePtr &, RobotTrajectory, getFirstWayPointPtr)(RobotTrajectoryPtr *ptr)
 {
  return (*ptr)->getFirstWayPointPtr();
 }

MOVIMP( void , RobotTrajectory, getWayPointDurations)(RobotTrajectoryPtr *ptr)
 {
 std::deque<double> test= (*ptr)->getWayPointDurations();
 //return
 }

/** @brief  Returns the duration after start that a waypoint will be reached.
  *  @param  The waypoint index.
  *  @return The duration from start; retuns -1.0 if index is out of range.
  */
MOVIMP(double, RobotTrajectory, getWaypointDurationFromStart)(RobotTrajectoryPtr *ptr, std::size_t index)
{
  return (*ptr)->getWaypointDurationFromStart(index);
}


MOVIMP(double, RobotTrajectory, getWayPointDurationFromPrevious)(RobotTrajectoryPtr *ptr,std::size_t index)
 {
 return (*ptr)->getWayPointDurationFromPrevious(index);
 }

MOVIMP(void, RobotTrajectory, setWayPointDurationFromPrevious)(RobotTrajectoryPtr *ptr, std::size_t index, double value)
 {
  (*ptr)->setWayPointDurationFromPrevious(index, value);
 }

/**
 * \brief Add a point to the trajectory
 * \param state - current robot state
 * \param dt - duration from previous
 */
MOVIMP(void, RobotTrajectory, addSuffixWayPoint)(RobotTrajectoryPtr *ptr,const robot_state::RobotState &state, double dt)
{
 (*ptr)->addSuffixWayPoint(state, dt);
}

MOVIMP(void, RobotTrajectory, addPrefixWayPoint)(RobotTrajectoryPtr *ptr,const robot_state::RobotStatePtr &state, double dt)
 {
  (*ptr)->addPrefixWayPoint(state, dt);
 }

MOVIMP(void , RobotTrajectory,insertWayPoint)(RobotTrajectoryPtr *ptr,std::size_t index, const robot_state::RobotState &state, double dt)
 {
  (*ptr)->insertWayPoint(index, state, dt);
 }

 /**
  * \brief Add a trajectory to the end of the current trajectory
  * \param source - the trajectory to append to the end of current trajectory
  * \param dt - time step between last traj point in current traj, and first traj point of new traj
  */
MOVIMP(void, RobotTrajectory, append)(RobotTrajectoryPtr *ptr, const RobotTrajectoryPtr *source, double dt)
{
 (*ptr)->append(**source,dt);
}

MOVIMP(void, RobotTrajectory, swap)(RobotTrajectoryPtr *ptr,robot_trajectory::RobotTrajectory &other)
{
 (*ptr)->swap(other);
}

MOVIMP(void, RobotTrajectory, clear)(RobotTrajectoryPtr *ptr)
{
 (*ptr)->clear();
}

MOVIMP(double, RobotTrajectory, getAverageSegmentDuration)(RobotTrajectoryPtr *ptr)
{
 return (*ptr)->getAverageSegmentDuration();
}

/*
MOVIMP(void, getRobotTrajectoryMsg)(RobotTrajectoryPtr *ptr, moveit_msgs::RobotTrajectory &trajectory)
{
 (*ptr)->getRobotTrajectoryMsg(trajectory);
}

MOVIMP(void, setRobotTrajectoryMsg)(RobotTrajectoryPtr *ptr,const robot_state::RobotState &reference_state,
                            const trajectory_msgs::JointTrajectory &trajectory);

MOVIMP(void, setRobotTrajectoryMsg)(RobotTrajectoryPtr *ptr,const robot_state::RobotState &reference_state,
                            const moveit_msgs::RobotTrajectory &trajectory);

MOVIMP(void, setRobotTrajectoryMsg)(RobotTrajectoryPtr *ptr,const robot_state::RobotState &reference_state, const moveit_msgs::RobotState &state,
                            const moveit_msgs::RobotTrajectory &trajectory);
*/

MOVIMP(void, RobotTrajectory, reverse)(RobotTrajectoryPtr *ptr)
{
 (*ptr)->reverse();
}

MOVIMP(void, RobotTrajectory, unwind)(RobotTrajectoryPtr *ptr)
{
 (*ptr)->unwind();
}

/*MOVIMP(void, RobotTrajectory, unwind)(RobotTrajectoryPtr *ptr, const robot_state::RobotState &state)
{
 (*ptr)->unwind(state);
}*/

 /** @brief Finds the waypoint indicies before and after a duration from start.
  *  @param The duration from start.
  *  @param The waypoint index before the supplied duration.
  *  @param The waypoint index after (or equal to) the supplied duration.
  *  @param The progress (0 to 1) between the two waypoints, based on time (not based on joint distances).
  */
MOVIMP(void, RobotTrajectory, findWayPointIndicesForDurationAfterStart)(RobotTrajectoryPtr *ptr,const double &duration, int &before, int &after, double &blend)
{
 (*ptr)->findWayPointIndicesForDurationAfterStart(duration, before, after, blend);
}
