#include <ros/ros.h>
#include "dynamic_view_planning/test.hpp"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "test_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    debugging::testObject test(nh, nh_priv);


    //ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

    ROS_INFO("Test node is ready");

    ros::spin();
    return 0;

}
