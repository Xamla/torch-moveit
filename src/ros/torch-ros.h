#ifndef torch_ros_h
#define torch_ros_h

extern "C" {
#include <TH/TH.h>
}

#include <ros/ros.h>

#define ROSIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(ros_, class_name, _, name)

#endif  // torch_ros_h
