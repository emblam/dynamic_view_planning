#include <ros/ros.h>
#include "dynamic_view_planning/ig_server.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ig_server_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    dynamic_view_planning::IGCalculator ig_server(nh,nh_priv);

    ROS_INFO("Server node for Information Gain calculations is ready.");

    ros::spin();
    return 0;
}