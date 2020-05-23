#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "my_tutorial/misc.h"


namespace misc{

void displayCollisionInfo(planning_scene_monitor::PlanningSceneMonitorPtr& psm,
                            CollisionType ctype){

    std::string collision_type;
    switch(ctype){
        case CollisionType::SELF:
            collision_type = "SELF";
            break;
        case CollisionType::FULL:
            collision_type = "FULL";
            break;
    }

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    if (collision_type == "SELF"){
        planning_scene_monitor::LockedPlanningSceneRO(psm)->checkSelfCollision(
                                                        collision_request, collision_result);

    }else if(collision_type == "FULL"){
        planning_scene_monitor::LockedPlanningSceneRO(psm)->checkCollision(
                                                        collision_request, collision_result);
    }
    
    ROS_INFO_STREAM("><><><><><> " << collision_type <<" collision information <><><><><><");
    ROS_INFO_STREAM("Collision Test: Current state is " 
                    << (collision_result.collision ? "in " : "NOT in ") << collision_type << " collision");
    ROS_INFO_STREAM("distance: " << collision_result.distance);
    ROS_INFO_STREAM("number of contacts: " << collision_result.contact_count);
  
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
    {
        ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
    }

  // see the bodies involved in the contact
//   for(auto& x : collision_result.contacts){
//         ROS_INFO_STREAM("first body: " << x.first.first << "   second body: " << x.first.second);
    
//         //contact_information.contacts.push_back(x);
//         for(collision_detection::Contact c : x.second){
//             ROS_INFO_STREAM("first body: " << c.body_name_1 << "   second body:" << c.body_name_2);
//         }
//   }

ROS_INFO_STREAM("----------------------------------------------------");

}




}