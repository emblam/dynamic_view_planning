#include <ros/ros.h>
#include "dynamic_view_planning/ufomap_ig_calculator.hpp"
#include "ig_active_reconstruction_ros/world_representation_ros_server_ci.hpp"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "ig_calculator_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    boost::shared_ptr<ig_active_reconstruction::world_representation::CommunicationInterface> ig_calculator{new ufomap_ig::UFOMapIGCalculator{nh,nh_priv}};

    ig_active_reconstruction::world_representation::RosServerCI<boost::shared_ptr> ig_server(nh, ig_calculator);

    ROS_INFO("Information gain calculator is ready.");

    ros::spin();

    return 0;

}