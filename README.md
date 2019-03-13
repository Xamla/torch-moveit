# torch-moveit

Implementation of a [lua](http://www.lua.org/)/[Torch](http://torch.ch/) wrapper for the [ROS](http://www.ros.org/) manipulation software package [MoveIt!](http://moveit.ros.org/).
It is currently used by the Xamla team to move a UR5 robot equipped with RGB and depth camera to capture point clouds for scene understanding experiments.

## Functionality

- Planning and execution of robot movements from start to goal poses/states can be carried out via a wrapper of the [MoveGroupInterface](http://docs.ros.org/api/moveit_ros_planning_interface/html/move__group__interface_8h.html) class.
- Wrappers for [PlanningScene](http://docs.ros.org/api/moveit_core/html/classplanning__scene_1_1PlanningScene.html) and [World](http://docs.ros.org/api/moveit_core/html/classcollision__detection_1_1World.html) are in preparation.


## Mini demo

```
moveit = require 'moveit'
ros = moveit.ros

ros.init()
local sp = ros.AsyncSpinner()
sp:start()

g = moveit.MoveGroupInterface('arm')
pos, rot = g:getCurrentPose_Tensors()
print('pose:'..tostring(pos)..tostring(rot))
g:setPositionTarget(0.3, 0.5, 0.2)
s, p = g:plan()
g:execute(p)
```

## Warning / Disclaimer

This prototype of a torch-moveit wrapper is not intended for production use! It features absolutely no validation of arguments and pointers - if wrong parameters are passed the process will likely crash or behave unexpectedly.
The Xamla team hopes to provide a more reliable (and professionally supported) MoveIt! wrapper later in 2016.

## Job

If you are a robot guru or deep learning hero that is looking for a job, please check our [job offers](http://xamla.com/jobs/) (German). ;-)
