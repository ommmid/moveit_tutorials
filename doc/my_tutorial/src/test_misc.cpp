#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <ros/ros.h>

#include "my_tutorial/misc.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_pair");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  // planning_scene::PlanningScene planning_scene(kinematic_model);

  // misc::getSelfCollisionInformation(planning_scene);


  ros::shutdown();
  return 0;
}

