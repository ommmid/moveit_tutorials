

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv)
{
  const std::string NODE_NAME = "publisher_to_panda";
  ros::init(argc, argv, NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  ros::Publisher joint_pub = node_handle.advertise<sensor_msgs::JointState>("/joint_states", 10);
  ros::Rate loop_rate(10);

    sensor_msgs::JointState joint_state_msg;

    std::vector<std::string> joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6",
                                           "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"};

    std::vector<double> joint_positions = {0.2, 0.2, 0.2, -0.2, 0.2, 0.2, 0, 0.2, 0.2};
    // velocity: []
    // effort: []

    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name.resize(9);
    joint_state_msg.position.resize(9);

    auto pos = joint_positions.begin();
    auto nam = joint_names.begin();
    while(pos != joint_positions.end())
    {
        std::cout << "name: " << *nam << "     position: " << *pos << std::endl;
        joint_state_msg.name.push_back(*nam);
        joint_state_msg.position.push_back(*pos);
        ++pos;
        ++nam;
    }

  int count = 0;
  while (count < 100)
  {
    std::cout << "count: " << count << std::endl;

    joint_pub.publish(joint_state_msg);

    // ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

}