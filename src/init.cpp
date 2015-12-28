#include "torch-moveit.h"

#include <moveit/move_group_interface/move_group.h>
/*#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>*/

class TorchMoveItModule
{
public:
  TorchMoveItModule()
  {
    int argc = 0; 
    char** argv = 0;
    ros::init(argc, argv, "TorchMoveItModule");
    spinner.reset(new ros::AsyncSpinner(1));
    spinner->start();
  }
  
  ~TorchMoveItModule()
  {
    ros::shutdown();  
  }
  
  ros::AsyncSpinner& getSpinner()
  {
    return *spinner;
  }
  
private:
  std::auto_ptr<ros::AsyncSpinner> spinner;
};

typedef boost::shared_ptr<TorchMoveItModule> MoveItModulePtr;

MOVIMP(MoveItModulePtr*, TorchMoveItModule, new)()
{
   return new MoveItModulePtr(new TorchMoveItModule());
}

MOVIMP(void, TorchMoveItModule, delete)(MoveItModulePtr *ptr)
{
  delete ptr;
}
