#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

#include "dynamic_view_planning/view_space.hpp"
#include <tf2_ros/static_transform_broadcaster.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <ufomap/octree.h>
#include <ufomap_msgs/Ufomap.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <std_msgs/String.h>

#include "dynamic_view_planning/view_space.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <future>

int main(int argc, char** argv)
{
    ros::init(argc,argv, "test_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ROS_INFO("Test node is ready");

    std::string ig_id = "random";

    ROS_INFO("Reconstructed map generated.");

    ros::spin();
    return 0;
}
