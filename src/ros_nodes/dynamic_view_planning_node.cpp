#include "dynamic_view_planning/view_planner.hpp"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~"); 

    dynamic_view_planning::ViewPlanner planner(nh,nh_priv);

    ROS_INFO("View planner is running.");

    ros::spin();
    return 0;
}