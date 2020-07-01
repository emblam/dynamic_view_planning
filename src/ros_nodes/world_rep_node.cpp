#include <ros/ros.h>
#include "dynamic_view_planning/ufomap_ig_calculator.hpp"
#include "ig_active_reconstruction_ros/world_representation_ros_server_ci.hpp"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "world_rep_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    boost::shared_ptr<ig_active_reconstruction::world_representation::CommunicationInterface> ig_calculator{new ufomap_ig::UFOMapIGCalculator{nh,nh_priv}};

    ig_active_reconstruction::world_representation::RosServerCI<boost::shared_ptr> ig_server(nh, ig_calculator);

    ROS_INFO("World representation is setup.");

    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    return 0;

}