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

MOVIMP(void, RobotTrajectory, getWayPoint)(RobotTrajectoryPtr *ptr, int index, RobotStatePtr *out)
 {
  out->reset(new robot_state::RobotState((*ptr)->getWayPoint(index)));
 }

MOVIMP(void, RobotTrajectory, getLastWayPoint)(RobotTrajectoryPtr *ptr, RobotStatePtr *out)
 {
  out->reset(new robot_state::RobotState((*ptr)->getLastWayPoint()));
 }

MOVIMP(void, RobotTrajectory, getFirstWayPoint)(RobotTrajectoryPtr *ptr, RobotStatePtr *out)
 {
  out->reset(new robot_state::RobotState((*ptr)->getFirstWayPoint()));
 }

MOVIMP( void , RobotTrajectory, getWayPointDurations)(RobotTrajectoryPtr *ptr, THDoubleTensor *output)
 {
  std::deque<double> v= (*ptr)->getWayPointDurations();
  THDoubleTensor_resize1d(output, v.size());
  THDoubleTensor* output_ = THDoubleTensor_newContiguous(output);
  std::copy(v.begin(), v.end(), THDoubleTensor_data(output_));
  THDoubleTensor_freeCopyTo(output_, output);
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

MOVIMP(void, RobotTrajectory, setWayPointDurationFromPrevious)(RobotTrajectoryPtr *ptr, int index, double value)
 {
  (*ptr)->setWayPointDurationFromPrevious(index, value);
 }

/**
 * \brief Add a point to the trajectory
 * \param state - current robot state
 * \param dt - duration from previous
 */
MOVIMP(void, RobotTrajectory, addSuffixWayPoint)(RobotTrajectoryPtr *ptr, RobotStatePtr *state, double dt)
{
 (*ptr)->addSuffixWayPoint(**state, dt);
}

MOVIMP(void, RobotTrajectory, addPrefixWayPoint)(RobotTrajectoryPtr *ptr,RobotStatePtr *state, double dt)
 {
  (*ptr)->addPrefixWayPoint(**state, dt);
 }

MOVIMP(void , RobotTrajectory,insertWayPoint)(RobotTrajectoryPtr *ptr,int index, RobotStatePtr *state, double dt)
 {
  (*ptr)->insertWayPoint(index, **state, dt);
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

MOVIMP(void, RobotTrajectory, swap)(RobotTrajectoryPtr *ptr,const RobotTrajectoryPtr *other)
{
 (*ptr)->swap(**other);
}

MOVIMP(void, RobotTrajectory, clear)(RobotTrajectoryPtr *ptr)
{
 (*ptr)->clear();
}

MOVIMP(double, RobotTrajectory, getAverageSegmentDuration)(RobotTrajectoryPtr *ptr)
{
 return (*ptr)->getAverageSegmentDuration();
}

MOVIMP(void, RobotTrajectory,getRobotTrajectoryMsg)(RobotTrajectoryPtr *ptr, THByteStorage *output)
{
  moveit_msgs::RobotTrajectory trajectory;
 (*ptr)->getRobotTrajectoryMsg(trajectory);

 uint32_t length = ros::serialization::serializationLength(trajectory);
 THByteStorage_resize(output, length + sizeof(uint32_t));
 ros::serialization::OStream stream(THByteStorage_data(output), length + sizeof(uint32_t));
 stream.next((uint32_t)length);
 ros::serialization::serialize(stream, trajectory);
}

MOVIMP(void, RobotTrajectory,setRobotTrajectoryMsg)(RobotTrajectoryPtr *ptr,const RobotStatePtr *reference_state, THByteStorage *serialized_msg)
{
  moveit_msgs::RobotTrajectory rt_msg;
  // deserialize to moveit_msgs::RobotTrajectory message
  long storage_size = THByteStorage_size(serialized_msg);

  uint8_t *data = THByteStorage_data(serialized_msg);

  ros::serialization::IStream stream(data+ sizeof(uint32_t), static_cast<uint32_t>(storage_size-sizeof(uint32_t)));
  ros::serialization::Serializer<moveit_msgs::RobotTrajectory>::read(stream, rt_msg);
  (*ptr)->setRobotTrajectoryMsg(**reference_state, rt_msg);
}


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
