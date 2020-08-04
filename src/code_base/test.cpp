#include "dynamic_view_planning/test.hpp"
#include "std_msgs/String.h"

namespace debugging
{
testObject::testObject(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : nh_(nh)
    , nh_priv_(nh_priv)
{
    // cloud_in_ = std::string("/camera_front/depth/color/points");
	// cloud_sub_ = nh_.subscribe(cloud_in_,10, &testObject::testCallback,this);

    // std::string debug_in = "chatter";
	debug_sub_ = nh_.subscribe("chatter",10, &testObject::debugCallback,this);

    ROS_INFO("testObject constructor done");
}

// void testObject::testCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
// {
//     ROS_INFO("Callback!");
// }

void testObject::debugCallback(const std_msgs::String& msg)
{
    ROS_INFO("Callback from debug!");
}
}