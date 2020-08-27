#pragma once

#include <ros/ros.h>

#include "ufomap/octree.h"

#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

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

    ros::Duration transform_timeout_ = ros::Duration(10);

    ufomap::Octree reference_map_;

    std::string experiment_name_;

    std::string data_bag_;
    std::string reference_bag_;

   	unsigned int insert_depth_;
	unsigned int insert_n_;
    float max_range_;
};
}