#include <ros/ros.h>
#include "dynamic_view_planning/virtual_robot.hpp"
#include "ig_active_reconstruction/robot_communication_interface.hpp"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "virtual_robot_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    // Skapa robot, lägg till länkar till RosServer

    ROS_INFO("Virtual robot is running.");

    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    return 0;
}