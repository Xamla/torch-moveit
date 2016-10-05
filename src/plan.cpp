#include "torch-moveit.h"
#include <ros/serialization.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include "utils.h"

MOVIMP(PlanPtr*, Plan, new)()
{
   return new PlanPtr(new moveit::planning_interface::MoveGroup::Plan());
}

MOVIMP(void, Plan, delete)(PlanPtr *ptr)
{
  if (ptr)
    delete ptr;
}

MOVIMP(void, Plan, release)(PlanPtr *ptr)
{
  ptr->reset();
}

MOVIMP(void, Plan, getStartStateMsg)(PlanPtr *ptr, THByteStorage *output)
{
  const moveit_msgs::RobotState& start_state = (*ptr)->start_state_;
  uint32_t length = ros::serialization::serializationLength(start_state);
  THByteStorage_resize(output, length + sizeof(uint32_t));
  ros::serialization::OStream stream(THByteStorage_data(output), length + sizeof(uint32_t));
  stream.next((uint32_t)length);
  ros::serialization::serialize(stream, start_state);
}

MOVIMP(void, Plan, getTrajectoryMsg)(PlanPtr *ptr, THByteStorage *output)
{
  const moveit_msgs::RobotTrajectory& trajectory = (*ptr)->trajectory_;
  uint32_t length = ros::serialization::serializationLength(trajectory);
  THByteStorage_resize(output, length + sizeof(uint32_t));
  ros::serialization::OStream stream(THByteStorage_data(output), length + sizeof(uint32_t));
  stream.next((uint32_t)length);
  ros::serialization::serialize(stream, trajectory);
}


MOVIMP(int32_t, Plan, setTrajectoryMsg)(PlanPtr *ptr, THByteStorage *serialized_msg, int32_t offset)
{

  // deserialize to moveit_msgs::RobotTrajectory message
  long storage_size = THByteStorage_size(serialized_msg);

  uint8_t *data = THByteStorage_data(serialized_msg);

  ros::serialization::IStream stream(data+ sizeof(uint32_t), static_cast<uint32_t>(storage_size-sizeof(uint32_t)));
  moveit_msgs::RobotTrajectory rt_msg;
  ros::serialization::Serializer<moveit_msgs::RobotTrajectory>::read(stream, rt_msg);
  (*ptr)->trajectory_ = rt_msg;
  return static_cast<int32_t>(stream.getData() - data);   // return new offset
}


MOVIMP(double, Plan, getPlanningTime)(PlanPtr *ptr)
{
  return (*ptr)->planning_time_;
}
