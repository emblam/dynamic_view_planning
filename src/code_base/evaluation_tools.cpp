#include "dynamic_view_planning/evaluation_tools.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <fstream>

#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include <std_msgs/String.h>

#include <future>
namespace evaluation_tools
{

EvaluationPortal::EvaluationPortal(ros::NodeHandle nh, ros::NodeHandle nh_priv)
    : nh_(nh)
    , nh_priv_(nh_priv)
    , tf_listener_(tf_buffer_)
    , experiment_name_(nh_priv.param("experiment_name", std::string("test")))
    , insert_depth_(nh_priv.param("insert_depth", 3))
	, insert_n_(nh_priv.param("insert_n", 0))
	, max_range_(nh_priv.param("max_range", 2))
{
    // Reference bag
    reference_bag_ = "/home/rpl/bagfiles/" + experiment_name_ + "_reference.bag";
    
    analyse_file_server_ = nh_.advertiseService("analyse_file", &EvaluationPortal::analyseFile, this);
        
}

bool EvaluationPortal::analyseFile(dynamic_view_planning::AnalyseFile::Request &req, dynamic_view_planning::AnalyseFile::Response &)
{
    std::string input_file = "/home/rpl/bagfiles/" + req.input_file + ".bag";
    std::string output_file = "/home/rpl/results/" + req.output_file + ".txt"; 

    std::ofstream result_file(output_file.c_str());

    rosbag::Bag recon_bag;
    recon_bag.open(input_file, rosbag::bagmode::Read);

    rosbag::Bag ref_bag;
    ref_bag.open(reference_bag_);

    std::vector<std::string> topics;
    topics.push_back(std::string("/reconstructed_map"));

    ros::Duration half_duration(0.5);

    for(rosbag::MessageInstance const m: rosbag::View(recon_bag, rosbag::TopicQuery(topics)))
    {
        ufomap_msgs::Ufomap::ConstPtr recon_msg = m.instantiate<ufomap_msgs::Ufomap>();

        ufomap::Octree recon_map;
        ufomap_msgs::msgToMap(*recon_msg,recon_map);

        ros::Time msg_time = recon_msg->header.stamp;

        ros::Time start_time = msg_time - half_duration; 
        ros::Time end_time = msg_time + half_duration;

        ros::Duration threshold = ros::Duration(1);

        ufomap_msgs::Ufomap::ConstPtr ref_msg;

        for(rosbag::MessageInstance const ref_m: rosbag::View(ref_bag, start_time, end_time, true))
        {
            ufomap_msgs::Ufomap::ConstPtr temp_msg = ref_m.instantiate<ufomap_msgs::Ufomap>();

            ros::Time temp_time = temp_msg->header.stamp;
            ros::Duration temp_threshold = temp_time - msg_time;
            
            if(temp_threshold < threshold)
            {
                threshold = temp_threshold;
                ref_msg = temp_msg;
            }
        }

        ufomap::Octree ref_map;
        ufomap_msgs::msgToMap(*ref_msg, ref_map);

        std::string results = EvaluationPortal::compareMaps(recon_map, ref_map);
        
        result_file << results;
    }

    result_file.close();

    return true;
}
 
std::string EvaluationPortal::compareMaps(ufomap::Octree reconstruction, ufomap::Octree reference)
{   
    ufomap_math::Vector3 min_corner = Vector3(0, 0.10, 0);
    ufomap_math::Vector3 max_corner = Vector3(0.4, 0.5, 0.4);

    ufomap_geometry::AABB roi = ufomap_geometry::AABB(min_corner, max_corner);

    double true_positives;
    double false_positives;
    double ref_occupied;
    double recon_occupied;
    double no_voxels;
    double no_unknown;

    for (auto it = reference.begin_leafs_bounding(roi, true, false, false, false, 0), 
        it_end = reference.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {
        ++ref_occupied;
        if(reconstruction.isOccupied(*it))
        {
            ++true_positives;
        }
    }

    for (auto it = reconstruction.begin_leafs_bounding(roi, true, false, false, false, 0), 
        it_end = reconstruction.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {
        ++recon_occupied;
        if(!reference.isOccupied(*it))
        {
            ++false_positives;
        }
    }

    for (auto it = reconstruction.begin_leafs_bounding(roi, true, true, true, false, 0), 
        it_end = reconstruction.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {
        ++no_voxels;
    }

    for (auto it = reconstruction.begin_leafs_bounding(roi, false, false, true, false, 0), 
        it_end = reconstruction.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {
        ++no_unknown;
    }

    double true_positive_rate = true_positives/ref_occupied;
    double false_positive_rate = false_positives/recon_occupied;
    double unknown_rate = no_unknown/no_voxels;

    std::string result = std::to_string(true_positive_rate) + std::string(",") + std::to_string(false_positive_rate) + std::string(",") + std::to_string(unknown_rate) + std::string("\n");
    
    return result;
} 
}