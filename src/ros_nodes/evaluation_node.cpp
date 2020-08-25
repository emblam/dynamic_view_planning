#include "ros/ros.h"
#include "dynamic_view_planning/evaluation_tools.hpp"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "evaluation_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~"); 

    evaluation_tools::EvaluationPortal evaluator(nh,nh_priv);

    ROS_INFO("Evaluation is running.");

    ros::spin();
    return 0;
}