/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "my_tutorial/misc.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "check_collision");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup
  static const std::string PLANNING_GROUP = "panda_arm";

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
 
  const std::string ROBOT_DESCRIPTION = "robot_description";
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

  //planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
  // planning_scene_monitor::PlanningSceneMonitor psm(planning_scene, ROBOT_DESCRIPTION);
  // psm.startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  // psm.startSceneMonitor();
  // planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

  // Create a planing scene monitor
  // planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(
      new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM("====== planning scene was not found");
  }else
  {
    ROS_INFO_STREAM("====== planning scene is loaded found");
  }
  

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor();
  planning_scene_monitor->startStateMonitor();

// psm.getPlanningScene()->getCurrentStateNonConst().setToRandomPositions();
  robot_state::RobotState& current_state = planning_scene_monitor->getPlanningScene()->getCurrentStateNonConst();
  current_state.update();

 // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      planning_scene_monitor->getPlanningScene()->getCurrentState().getJointModelGroup(PLANNING_GROUP);

  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->printKnownObjects(std::cout);

  ROS_INFO_STREAM("name of the model group: " << joint_model_group->getName());

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots, // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();
  // TODO: before visual_tools.trigger(), selfCollision returns the robot is in collision, why???

  visual_tools.prompt("Press 'next': Check Self Collision");

  // Check Collision
  // ^^^^^^^^^^^^^^^^^^^^
  // misc::displayCollisionInfo(planning_scene_monitor, misc::CollisionType::SELF);
  // misc::displayCollisionInfo(planning_scene_monitor, misc::CollisionType::FULL);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->
          checkSelfCollision(collision_request, collision_result);
  misc::displayCollisionInfo(collision_result, misc::CollisionType::SELF);
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->
          checkCollision(collision_request, collision_result);
  misc::displayCollisionInfo(collision_result, misc::CollisionType::FULL);


  visual_tools.prompt("Press 'next': Add object1");
  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  std::string planning_frame_name = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->getPlanningFrame();
  ROS_INFO_STREAM("planning frame: " << planning_frame_name);
  
  // Define collision object1, is NOT in collision
  moveit_msgs::CollisionObject collision_object1;
  collision_object1.header.frame_id = planning_frame_name; //move_group.getPlanningFrame();
  collision_object1.id = "box1";
  shape_msgs::SolidPrimitive primitive1;
  primitive1.type = primitive1.BOX;
  primitive1.dimensions.resize(3);
  primitive1.dimensions[0] = 0.4;
  primitive1.dimensions[1] = 0.1;
  primitive1.dimensions[2] = 0.4;
  geometry_msgs::Pose box1_pose;
  box1_pose.orientation.w = 1.0;
  box1_pose.position.x = 0.4;
  box1_pose.position.y = -0.2;
  box1_pose.position.z = 1.0;
  collision_object1.primitives.push_back(primitive1);
  collision_object1.primitive_poses.push_back(box1_pose);
  collision_object1.operation = collision_object1.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object1);

  visual_tools.prompt("Press 'next': Add object2");
  // Define collision object2, is in collision
  moveit_msgs::CollisionObject collision_object2;
  collision_object2.header.frame_id = planning_frame_name; //move_group.getPlanningFrame();
  collision_object2.id = "box2";
  shape_msgs::SolidPrimitive primitive2;
  primitive2.type = primitive2.BOX;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[0] = 0.4;
  primitive2.dimensions[1] = 0.1;
  primitive2.dimensions[2] = 0.4;
  geometry_msgs::Pose box2_pose;
  box2_pose.orientation.w = 1.0;
  box2_pose.position.x = 0.3;
  box2_pose.position.y = 0;
  box2_pose.position.z = 0.8;
  collision_object2.primitives.push_back(primitive2);
  collision_object2.primitive_poses.push_back(box2_pose);
  collision_object2.operation = collision_object2.ADD;

  collision_objects.push_back(collision_object2);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  current_state.update();
  planning_scene_monitor->updateSceneWithCurrentState();
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->printKnownObjects(std::cout);

  visual_tools.prompt("Press 'next': Check Collision");
  
  // check collision
  // misc::displayCollisionInfo(planning_scene_monitor, misc::CollisionType::SELF);
  // misc::displayCollisionInfo(planning_scene_monitor, misc::CollisionType::FULL);
  collision_result.clear();
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->
        checkSelfCollision(collision_request, collision_result);
  misc::displayCollisionInfo(collision_result, misc::CollisionType::SELF);
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->
        checkCollision(collision_request, collision_result);
  misc::displayCollisionInfo(collision_result, misc::CollisionType::FULL);

  // Wait for MoveGroup to recieve and process the collision object message
  visual_tools.prompt("Press 'next': remove object2 which is in collision");

  // remove collsion object2 which was in collision
  std::vector<std::string> remove_objects_ids = {"box2"};
  planning_scene_interface.removeCollisionObjects(remove_objects_ids);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "remove object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next': check collision");
  // ><><><><><> check collision <><><><><><
  // misc::displayCollisionInfo(planning_scene_monitor, misc::CollisionType::SELF);
  // misc::displayCollisionInfo(planning_scene_monitor, misc::CollisionType::FULL);
  collision_result.clear();
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->checkSelfCollision(collision_request, collision_result);
  misc::displayCollisionInfo(collision_result, misc::CollisionType::SELF);
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->checkCollision(collision_request, collision_result);
  misc::displayCollisionInfo(collision_result, misc::CollisionType::FULL);


  visual_tools.prompt("Press 'next': set to random position");
  // ---------- Set to random position
  // robot_state::RobotState& current_state = planning_scene_monitor->getPlanningScene()->getCurrentStateNonConst();
  current_state.setToRandomPositions();
  current_state.update();
  planning_scene_monitor->updateSceneWithCurrentState();
  
  visual_tools.prompt("Press 'next': Check collision");
  collision_result.clear();
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->checkSelfCollision(collision_request, collision_result);
  misc::displayCollisionInfo(collision_result, misc::CollisionType::SELF);
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->checkCollision(collision_request, collision_result);
  misc::displayCollisionInfo(collision_result, misc::CollisionType::FULL);

  visual_tools.prompt("Press 'next': Set the robot in self collision state");
  // ---------- Set the robot in self collision state
  // First, manually set the Panda arm to a position where we know
  // internal (self) collisions do happen. Note that this state is now
  // actually outside the joint limits of the Panda, which we can also
  // check for directly.
  std::vector<double> joint_values = { 0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0 };
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  planning_scene_monitor->updateSceneWithCurrentState();
  current_state.update();
  ROS_INFO_STREAM("Current state should be in self collision "
                  << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

  visual_tools.prompt("Press 'next': Check collision");
  collision_result.clear();
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->checkSelfCollision(collision_request, collision_result);
  misc::displayCollisionInfo(collision_result, misc::CollisionType::SELF);
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->checkCollision(collision_request, collision_result);
  misc::displayCollisionInfo(collision_result, misc::CollisionType::FULL);


  visual_tools.prompt("Press 'next': end the tutorial");

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
