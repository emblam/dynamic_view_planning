#pragma once

#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <ufomap/octree.h>
#include <ufomap_msgs/Ufomap.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include "dynamic_view_planning/view_space.hpp"

#include "dynamic_view_planning_msgs/RequestIG.h"

namespace dynamic_view_planning
{
class ViewPlanner
{
public: 

    ViewPlanner(ros::NodeHandle nh, ros::NodeHandle nh_priv);

private: 
    
    void collectData();

    void evaluateViews();

public: 

private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    std::string current_position_frame_;

    ros::ServiceClient request_ig_;

    std::string state_;
    std::string input_topic_;

    unsigned int insert_depth_;
	unsigned int insert_n_;
    float max_range_;
    bool force_movement_;

    std::string experiment_name_;
    std::string ig_;
    std::string data_bag_name_;
    std::string reconstructed_bag_name_;
    std::string initial_view_;

    dynamic_view_planning::ViewSpace view_space_;
    ufomap::Octree reconstructed_map_;

    ros::Time current_time_;
    ros::Duration time_step_;
    ros::Time start_time_;
    ros::Time end_time_;

    rosbag::Bag data_bag_;
    rosbag::Bag reconstructed_bag_;

    //Logfiles for debug&results

    std::string position_log_name_;
    std::string ig_log_name_;

    std::ofstream position_log_file_;
    std::ofstream ig_log_file_;

};
}