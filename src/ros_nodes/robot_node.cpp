#include <ros/ros.h>
#include "dynamic_view_planning/virtual_robot.hpp"
#include "ig_active_reconstruction/robot_communication_interface.hpp"
#include <ig_active_reconstruction_ros/robot_ros_server_ci.hpp>

int main(int argc, char **argv)
{
    ros::init(argc,argv, "virtual_robot_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    boost::shared_ptr<ig_active_reconstruction::robot::CommunicationInterface> virtual_robot{new robot::virtualRobot{nh,nh_priv}};

    ig_active_reconstruction::robot::RosServerCI robot_server(nh, virtual_robot);

    ROS_INFO("Virtual robot is running.");

    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    return 0;
}