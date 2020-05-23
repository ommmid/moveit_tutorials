#include <ros/ros.h>
#include <map>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_pair");
  ros::AsyncSpinner spinner(1);
  spinner.start();


    typedef std::map<std::pair<std::string, double>, std::vector<double> > ContactMap;

    std::pair<std::string, double> pair_1 ("str1", 12.3);
    ROS_INFO_STREAM("pair 1: " << pair_1.first);

    std::map<std::string, double> map_1;
    map_1["m"] = 1.23;
    ROS_INFO_STREAM("map 1: " << map_1["m"]);

    std::vector<double> vec = {96, 45, 26, 23};

    ContactMap contact_map = {
        {pair_1, vec}
    } ;

/* auto& it = contact_map.begin();
    ROS_INFO_STREAM("from map:" << it.first);
*/
    for(auto& x : contact_map){
        ROS_INFO_STREAM( x.first.first << " :  " << x.first.second);
        ROS_INFO_STREAM( x.second[0]);
    }

  ros::shutdown();
  return 0;
}
