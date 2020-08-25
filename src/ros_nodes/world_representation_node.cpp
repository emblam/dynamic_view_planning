#include <ros/ros.h>
#include "dynamic_view_planning/ufomap_world_representation.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "world_representation_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    dynamic_ufomapping::UFOMapWorld world(nh,nh_priv);

    ROS_INFO("UFOMap world representation is ready.");

    ros::spin();
    return 0;
}