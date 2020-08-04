#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"

namespace debugging
{
class testObject
{
public: 

    testObject(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:

    // void testCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void debugCallback(const std_msgs::String& msg);

private:

    ros::NodeHandle& nh_;
    ros::NodeHandle& nh_priv_;
    
    // ros::Subscriber cloud_sub_;
    // std::string cloud_in_;

    ros::Subscriber debug_sub_;

};
}