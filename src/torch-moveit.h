#ifndef torch_moveit_h
#define torch_moveit_h

extern "C" {
#include <TH/TH.h>
}

#include <boost/shared_ptr.hpp>
#include <moveit/move_group_interface/move_group.h>

typedef boost::shared_ptr<std::vector<std::string> > StringsPtr;
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup::Plan> PlanPtr;
typedef boost::shared_ptr<moveit::core::RobotState> RobotStatePtr;

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
