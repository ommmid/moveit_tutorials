#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/planning_scene/planning_scene.h>


namespace misc{

// can I make a method without puting it in any class, straight inside namespace

struct ContactInformation{

    bool in_collision;
    double closest_destance;
    std::size_t contact_count;

    std::vector<std::string> first_bodies;
    std::vector<std::string> second_bodies;
    std::vector<collision_detection::Contact> contacts;
};

enum CollisionType
    {
        SELF,
        FULL
    };

void displayCollisionInfo(planning_scene_monitor::PlanningSceneMonitorPtr& psm,
                            CollisionType ctype);

void displayCollisionInfo(collision_detection::CollisionResult& collision_result, CollisionType ctype);
}

