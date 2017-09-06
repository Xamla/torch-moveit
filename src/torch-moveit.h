#ifndef torch_moveit_h
#define torch_moveit_h

extern "C" {
#include <TH/TH.h>
}

//#include <boost/shared_ptr.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <tf_conversions/tf_eigen.h>

typedef std::vector<std::string> StringVector;
typedef std::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupInterfacePtr;
typedef std::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;
typedef std::shared_ptr<moveit::planning_interface::MoveGroup::Plan> PlanPtr;
typedef std::shared_ptr<moveit::core::RobotState> RobotStatePtr;
typedef robot_model::RobotModelPtr RobotModelPtr;
typedef std::shared_ptr<robot_trajectory::RobotTrajectory> RobotTrajectoryPtr;
typedef std::shared_ptr<trajectory_processing::IterativeParabolicTimeParameterization> IptPtr;

#define MOVIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(moveit_, class_name, _, name)

class MoveItWrapperException
  : public std::runtime_error
{
public:
  MoveItWrapperException(const std::string& reason)
    : runtime_error(reason)
  {
  }
};

#endif // torch_moveit_h
