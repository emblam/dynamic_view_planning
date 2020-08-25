#pragma once

#include <ros/ros.h>

#include "ufomap/octree.h"

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "dynamic_view_planning/view_space.hpp"

#include "dynamic_view_planning/AnalyseFile.h"

namespace evaluation_tools
{
class EvaluationPortal
{
public: 
    
    EvaluationPortal(ros::NodeHandle nh, ros::NodeHandle nh_priv);

private: 

    bool analyseFile(dynamic_view_planning::AnalyseFile::Request &req, dynamic_view_planning::AnalyseFile::Response &res);    

    std::string compareMaps(ufomap::Octree reconstruction, ufomap::Octree reference);
public: 

private: 

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    ros::ServiceServer analyse_file_server_;

    std::string input_topic_left_;
    std::string input_topic_right_;
    std::string input_topic_front_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Duration transform_timeout_ = ros::Duration(10);

    ufomap::Octree reference_map_;

    dynamic_view_planning::ViewSpace view_space_;

    std::string experiment_name_;

    std::string data_bag_;
    std::string reference_bag_;

   	unsigned int insert_depth_;
	unsigned int insert_n_;
    float max_range_;

    bool create_reference_;
};
}