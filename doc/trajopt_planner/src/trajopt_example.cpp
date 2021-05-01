#include <ros/ros.h>
#include <ros/console.h>
// #include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "moveit/planning_interface/planning_request.h"
#include <moveit/robot_model/robot_model.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>


/* Author: Omid Heidari
   Desc: This file is a test for using trajopt in MoveIt. The goal is to make different types of constraints in
   MotionPlanRequest and visualize the result calculated by using trajopt planner.
*/

template <typename K>
void printVector(std::string str, std::vector<K> v)
{
  std::stringstream ss;
  ss << str << " ";
  for (std::size_t i = 0; i < v.size(); ++i)
      ss << v[i] << " ";

  std::cout << ss.str() << std::endl;
}

void publish_joint_state(ros::NodeHandle& n, const std::vector<double>& position)
{
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
  ros::Rate loop_rate(50);

  sensor_msgs::JointState joint_state_msg;

  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_msg.name.resize(9);
  joint_state_msg.position.resize(9);

  joint_state_msg.name[0] = "panda_joint1";         joint_state_msg.position[0] = position[0];
  joint_state_msg.name[1] = "panda_joint2";         joint_state_msg.position[1] = position[1];
  joint_state_msg.name[2] = "panda_joint3";         joint_state_msg.position[2] = position[2];
  joint_state_msg.name[3] = "panda_joint4";         joint_state_msg.position[3] = position[3];
  joint_state_msg.name[4] = "panda_joint5";         joint_state_msg.position[4] = position[4];
  joint_state_msg.name[5] = "panda_joint6";         joint_state_msg.position[5] = position[5];
  joint_state_msg.name[6] = "panda_joint7";         joint_state_msg.position[6] = position[6];
  joint_state_msg.name[7] = "panda_finger_joint1";  joint_state_msg.position[7] = position[7];
  joint_state_msg.name[8] = "panda_finger_joint2";  joint_state_msg.position[8] = position[8];

  int count = 0;
  while (count < 200)
  {
    std::cout << "count: " << count << std::endl;

    joint_pub.publish(joint_state_msg);
    loop_rate.sleep();
    ++count;
  }
}

void publish_collision_object(ros::NodeHandle& n, const Eigen::Isometry3d& pos)
{
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  ros::Rate loop_rate(1);

  auto cube = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world"; // "world"; 
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;   //"box1"
  marker.type = cube;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pos.translation().x();   //0.3;       0;
  marker.pose.position.y = pos.translation().y();   //0.6;       0;
  marker.pose.position.z = pos.translation().z(); //0.75;    0.5;
  Eigen::Quaterniond q = (Eigen::Quaterniond)pos.linear();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  // marker.lifetime = ros::Duration();

  int count = 0;
  while(count < 5)
  {
    std::cout << "count: " << count << std::endl;

    marker_pub.publish(marker);
    loop_rate.sleep();
    ++count;
  }

}

void publish_planning_scene(ros::NodeHandle& n, const planning_scene::PlanningScenePtr& ps)
{
  ros::Publisher ps_pub = n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  ros::Rate loop_rate(1);

  moveit_msgs::PlanningScene scene;
  ps->getPlanningSceneMsg(scene);

  int count = 0;
  while(count < 5)
  {
    std::cout << "count: " << count << std::endl;

    ps_pub.publish(scene);
    loop_rate.sleep();
    ++count;
  }
}

int main(int argc, char** argv)
{
  const std::string NODE_NAME = "trajopt_example";
  ros::init(argc, argv, NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  const std::string PLANNING_GROUP = "panda_arm";
  const std::string ROBOT_DESCRIPTION = "robot_description";
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

  // Dont use planning scene monitor because I do not want a motion planner like trajopt to be dependent on it.
  // What we have access to in a motion planner is a const pointer to planning_scene from planning context
  // so the questions becomes how to use this PlanningSceneConstPtr to check collision for different state of the robot

  // make planning scene and robot state with robotmodel
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));

  // Create a planing scene monitor
  // planning_scene_monitor::PlanningSceneMonitorPtr psm(
  //     new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
  // psm->startSceneMonitor();
  // psm->startWorldGeometryMonitor();
  // psm->startStateMonitor();
  // Set the collision detectro to Bullet
  planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

  // Create JointModelGroup
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
  const std::vector<std::string>& link_model_names = joint_model_group->getLinkModelNames();
  ROS_INFO_NAMED(NODE_NAME, "end effector name %s\n", link_model_names.back().c_str());

  printVector("joint names: ", joint_names);

  // ----- change the robot state directly from the planning scne  
  std::vector<double> joint_values_initial = { 0.1, -0.2, 0, -0.356, 0, 0.5, 0.785 };
  planning_scene->getCurrentStateNonConst().setJointGroupPositions(joint_model_group, joint_values_initial);

  std::vector<double> copied_joint_values;
  planning_scene->getCurrentStateNonConst().copyJointGroupPositions(joint_model_group, copied_joint_values);
  printVector("copied joint values, initial?: ", copied_joint_values);

  // ----- chagne the robot state separately, does this have any effect on planning scene?
  std::vector<double> joint_values_A =  { 0, -0.71, 0, -2.2, 0, 1.49, 0.71 };
  robot_state->setJointGroupPositions(joint_model_group, joint_values_A);
  robot_state->update();

  robot_state::RobotState robot_state_A(*robot_state);

  // ----- no it does not. I have to update the scene through robot state
  planning_scene->setCurrentState(*robot_state);

  copied_joint_values.clear();
  planning_scene->getCurrentStateNonConst().copyJointGroupPositions(joint_model_group, copied_joint_values);
  printVector("copied joint values, A?: ", copied_joint_values);

  // ----- so what if I create a robot state pointer/alias that points to the planning scene's robot state
  // robot_state::RobotStatePtr robot_state_ps = std::make_shared<moveit::core::RobotState>(planning_scene->getCurrentStateNonConst());
  robot_state::RobotState& robot_state_ps = planning_scene->getCurrentStateNonConst();

  std::vector<double> joint_values_B = { -0.1, 1.2, 0, -2, 0, 0, 0 };
  robot_state_ps.setJointGroupPositions(joint_model_group, joint_values_B);
  robot_state_ps.update();

  copied_joint_values.clear();
  planning_scene->getCurrentStateNonConst().copyJointGroupPositions(joint_model_group, copied_joint_values);
  printVector("copied joint values, B?: ", copied_joint_values);

  // pointer did not work but updating the alias updated the robot state in planning scene

 /*
  // Initial state
  std::vector<double> initial_joint_values = { 0, -0.785, 0, -2.356, 0, 1.571, 0.785 };
  robot_state->setJointGroupPositions(joint_model_group, initial_joint_values);
  // robot_state->setToDefaultValues();
  robot_state->update();
  // we should not copy robotStatePtr but robotState itself if we want to capture this state for later uses
  robot_state::RobotState robot_state_initial(*robot_state);
  
  geometry_msgs::Pose pose_msg_current;
  const Eigen::Isometry3d& end_effector_transform_current = robot_state->getGlobalLinkTransform(link_model_names.back());
  pose_msg_current = tf2::toMsg(end_effector_transform_current);

  // Set the planner
  std::string planner_plugin_name = "trajopt_interface/TrajOptPlanner";
  node_handle.setParam("planning_plugin", planner_plugin_name);

  // Create pipeline
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
     new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));


  // Create response and request
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
*/
  // Set start state
  // ======================================================================================
  // panda_arm joint limits:
  //   -2.8973  2.8973
  //   -1.7628  1.7628
  //   -2.8973  2.8973
  //   -3.0718 -0.0698
  //   -2.8973  2.8973
  //   -0.0175  3.7525
  //   -2.8973  2.8973
/*
  std::vector<double> start_joint_values = { 0.4, 0.3, 0.5, -0.55, 0.88, 1.0, -0.075 };
  robot_state->setJointGroupPositions(joint_model_group, start_joint_values);
  robot_state->update();
  // this update in robot_state does not update the planning scene nor its monitor (psm). psm update
  // does not update the planning scene with new robot state either

  // std::vector<double> cur;
  // robot_state_initial.copyJointGroupPositions(joint_model_group, cur);
  // std::cout << "===>>> start state afte robot state updates: " << std::endl;
  // for(std::size_t k = 0; k < cur.size(); ++k)
  //   std::cout << cur[k] << " ";
  // std::cout << std::endl;

  req.start_state.joint_state_msg.name = joint_names;
  req.start_state.joint_state.position = start_joint_values;
  req.goal_constraints.clear();
  req.group_name = PLANNING_GROUP;

  geometry_msgs::Pose pose_msg_start;
  const Eigen::Isometry3d& end_effector_transform_start = robot_state->getGlobalLinkTransform(link_model_names.back());
  pose_msg_start = tf2::toMsg(end_effector_transform_start);
*/

 /*
  
  // Set the goal state 
  // ========================================================================================
  std::vector<double> goal_joint_values = { 0.8, 0.7, 1, -1.3, 1.9, 2.2, -0.1 };
  robot_state->setJointGroupPositions(joint_model_group, goal_joint_values);
  robot_state->update();
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(*robot_state, joint_model_group);
  req.goal_constraints.push_back(joint_goal);
  req.goal_constraints[0].name = "goal_pos";
  // Set joint tolerance
  std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = req.goal_constraints[0].joint_constraints;
  for (std::size_t x = 0; x < goal_joint_constraint.size(); ++x)
  {
    ROS_INFO_STREAM_NAMED(NODE_NAME ," ======================================= joint position at goal: " << goal_joint_constraint[x].position);
    req.goal_constraints[0].joint_constraints[x].tolerance_above = 0.001;
    req.goal_constraints[0].joint_constraints[x].tolerance_below = 0.001;
  }

  geometry_msgs::Pose pose_msg_goal;
  const Eigen::Isometry3d& end_effector_transform_goal = robot_state->getGlobalLinkTransform(link_model_names.back());
  pose_msg_goal = tf2::toMsg(end_effector_transform_goal);


// Reference Trajectory. The type should be defined in the yaml file.
  // ========================================================================================
  // type: STATIONARY
  // No need to pass any trajectory. The current joint values will be replicated for all timesteps

  // type: JOINT_INTERPOLATED
  // The joint values at a specified state. Could be the goal state or one of the goals when having multiple goal states
  // The first index (points[0]) is used to set the values of the joints at the specified state as follows:
  req.reference_trajectories.resize(1);
  req.reference_trajectories[0].joint_trajectory.resize(1);
  req.reference_trajectories[0].joint_trajectory[0].joint_names = joint_names;
  req.reference_trajectories[0].joint_trajectory[0].points.resize(1);
  req.reference_trajectories[0].joint_trajectory[0].points[0].positions = goal_joint_values;

  // type: GIVEN_TRAJ
  // For this example, we give an interpolated trajectory
  int const N_STEPS = 20;  // number of steps
  int const N_DOF = 7;     // number of degrees of freedom

  // We need one reference trajectory with one joint_trajectory
  req.reference_trajectories.resize(1);
  req.reference_trajectories[0].joint_trajectory.resize(1);
  // trajectory includes both the start and end points (N_STEPS + 1)
  req.reference_trajectories[0].joint_trajectory[0].points.resize(N_STEPS + 1);
  req.reference_trajectories[0].joint_trajectory[0].joint_names = joint_names;
  req.reference_trajectories[0].joint_trajectory[0].points[0].positions = initial_joint_values;

  std::vector<double> joint_values = initial_joint_values;
  // Increment joint values at each step
  for (std::size_t step_index = 1; step_index <= N_STEPS; ++step_index)
  {
    double increment;
    step_index <= 10 ? (increment = 0.05) : (increment = 0.044);
    for (int dof_index = 0; dof_index < N_DOF; ++dof_index)
    {
      joint_values[dof_index] = joint_values[dof_index] + increment;
    }
    req.reference_trajectories[0].joint_trajectory[0].joint_names = joint_names;
    req.reference_trajectories[0].joint_trajectory[0].points[step_index].positions = joint_values;
  }
*/
  // Visualization
  // ========================================================================================
  namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, psm);
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");

  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();  // clear all old markers
  visual_tools.trigger();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz 
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Learn Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations 
  visual_tools.trigger();

  // We can also use visual_tools to wait for user input 
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // add collision object
  // ========================================================================================
  visual_tools.prompt("Press 'next' to add collision object to plannin scene \n");

  // add the object to planning scene without using planning_scene_interface
  Eigen::Isometry3d box_pose_iso{ Eigen::Isometry3d::Identity() };
  box_pose_iso.translation().x() = 0;
  box_pose_iso.translation().y() = 0;
  box_pose_iso.translation().z() = 0.5;
  shapes::ShapePtr cube_shape = std::make_shared<shapes::Box>(0.2, 0.2, 0.2);
  planning_scene->getWorldNonConst()->addToObject("box1", cube_shape, box_pose_iso);

  visual_tools.prompt("Press 'next' to view collision object in rviz \n");
  publish_collision_object(node_handle, box_pose_iso);

  // set the state back to the initial
  robot_state_ps.setJointGroupPositions(joint_model_group, joint_values_initial);
  robot_state_ps.update();
  copied_joint_values.clear();
  planning_scene->getCurrentStateNonConst().copyJointGroupPositions(joint_model_group, copied_joint_values);
  printVector("copied joint values, initial?: ", copied_joint_values);
  robot_state::RobotState robot_state_initial(robot_state_ps);
  
  visual_tools.prompt("Press 'next' to see the robot at initial state \n");
  joint_values_initial.push_back(0.2);
  joint_values_initial.push_back(0.2);
  publish_joint_state(node_handle, joint_values_initial);

  planning_scene->setName("omid");
  // this visual_tools.prompt is very important too, I dont know why
  visual_tools.prompt("Press 'next' to see planning scene published \n");
  publish_planning_scene(node_handle, planning_scene); 
  visual_tools.prompt("Press 'next' to see the collision result at the initial state \n");
  // Check collision
  collision_detection::CollisionRequest collision_req;
  collision_detection::CollisionResult collision_res;
  collision_req.group_name = PLANNING_GROUP;
  collision_req.contacts = true;
  collision_req.max_contacts = 100;
  collision_req.max_contacts_per_pair = 5;
  collision_req.verbose = false;

  std::cout << "=====>>> planning scene info: " << std::endl;
  planning_scene->printKnownObjects();
  std::cout << "robot model: " << planning_scene->getRobotModel()->getName() << std::endl;
  std::cout << "collision checker name: " << planning_scene->getActiveCollisionDetectorName() << std::endl;
  copied_joint_values.clear();
  planning_scene->getCurrentStateNonConst().copyJointGroupPositions(joint_model_group, copied_joint_values);
  printVector("copied joint values, initial?: ", copied_joint_values);

  collision_res.clear();
  planning_scene->checkCollision(collision_req, collision_res);
  std::cout << "initial state is in collision? " << collision_res.collision << std::endl;
  std::cout << "number of contacts " << collision_res.contact_count << std::endl;
  for (auto it = collision_res.contacts.begin(); it != collision_res.contacts.end(); ++it)
      ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());

  // ------------------------------ diff()
  planning_scene::PlanningScenePtr ps = planning_scene->diff();
  // ps is a another shared_pointer created from planning_scene shared pointer
  // robot state is at initial
  ps->setName("omidiff");

  visual_tools.prompt("Press 'next' to see planning scene diff() published \n");
  publish_planning_scene(node_handle, ps); // can we publish collision too???????????????
  visual_tools.prompt("Press 'next' to see the collision result at the initial state \n");

  std::cout << "=====>>>  diff() info: " << std::endl;
  ps->printKnownObjects();
  std::cout << "robot model: " << ps->getRobotModel()->getName() << std::endl;
  std::cout << "collision checker name: " << ps->getActiveCollisionDetectorName() << std::endl;
  copied_joint_values.clear();
  ps->getCurrentStateNonConst().copyJointGroupPositions(joint_model_group, copied_joint_values);
  printVector("copied joint values, initial?: ", copied_joint_values);

  collision_res.clear();
  ps->checkCollision(collision_req, collision_res);
  std::cout << "initial state is in collision? " << collision_res.collision << std::endl;
  std::cout << "number of contacts " << collision_res.contact_count << std::endl;
  for (auto it = collision_res.contacts.begin(); it != collision_res.contacts.end(); ++it)
      ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());


  std::cout << "does the name of the planning scene that  pointer planning_scene_ps points to also changes to 'omidiff' ?" << std::endl;
  std::cout << planning_scene->getName() << std::endl;
  // apparently diff() is pointing to a new object different from the original planning_scene object

  visual_tools.prompt("Press 'next' do the planning \n");
/*
  // Solve the problem
  // ========================================================================================
  
  // --- move back the robot state to initial state
  robot_state->setJointGroupPositions(joint_model_group, initial_joint_values);
  robot_state->setToDefaultValues();
  robot_state->update();
  
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm); // lscene is of type PlanningSceneConstPtr

    // Now, call the pipeline and check whether planning was successful. 
    planning_pipeline->generatePlan(lscene, req, res);
  }
  // Check that the planning was successful 
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR_STREAM_NAMED(NODE_NAME, "Could not compute plan successfully");
    return 0;
  }

  visual_tools.prompt("Press 'next' to visualize the result");

  // Visualize the result
  // ========================================================================================
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  for (int timestep_index = 0; timestep_index < response.trajectory.joint_trajectory.points.size(); ++timestep_index)
  {
    std::stringstream joint_values_stream;
    for (int joint_index = 0;
         joint_index < response.trajectory.joint_trajectory.points[timestep_index].positions.size(); ++joint_index)
    {
      joint_values_stream << response.trajectory.joint_trajectory.points[timestep_index].positions[joint_index] << " ";
    }
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "step: " << timestep_index << " joints positions: " << joint_values_stream.str() );
  }
    
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);

  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  visual_tools.publishAxisLabeled(pose_msg_current, "current");
  visual_tools.publishText(text_pose, "current pose", rvt::WHITE, rvt::XLARGE);

  visual_tools.publishAxisLabeled(pose_msg_start, "start");
  visual_tools.publishText(text_pose, "start pose", rvt::BLUE, rvt::XLARGE);

  visual_tools.publishAxisLabeled(pose_msg_goal, "goal");
  visual_tools.publishText(text_pose, "goal pose", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' to finish demo \n");

  */
}

