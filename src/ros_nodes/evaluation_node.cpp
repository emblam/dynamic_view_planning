#include "ros/ros.h"
#include "dynamic_view_planning/ufomap_compare.hpp"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "evaluation_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv; 

    ROS_INFO("Evaluation is running.");

    ros::spin();
    return 0;
}