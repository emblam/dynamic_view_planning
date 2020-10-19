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
    , experiment_name_(nh_priv.param("experiment_name", std::string("test")))
    , insert_depth_(nh_priv.param("insert_depth", 0))
	, insert_n_(nh_priv.param("insert_n", 0))
	, max_range_(nh_priv.param("max_range", 2))
{
    // Reference bag
    reference_bag_ = "/home/rpl/bagfiles/" + experiment_name_ + "_reference.bag";
    
    analyse_file_server_ = nh_.advertiseService("analyse_file", &EvaluationPortal::analyseFile, this);
        
}

bool EvaluationPortal::analyseFile(dynamic_view_planning_msgs::AnalyseFile::Request &req, dynamic_view_planning_msgs::AnalyseFile::Response &)
{
    ROS_INFO("File analysis called.");
    std::string input_file = "/home/rpl/bagfiles/" + req.input_file + ".bag";
    std::string output_file = "/home/rpl/results/" + req.output_file + ".csv"; 

    std::ofstream result_file(output_file.c_str());

    rosbag::Bag recon_bag;
    recon_bag.open(input_file, rosbag::bagmode::Read);

    rosbag::Bag ref_bag;
    ref_bag.open(reference_bag_);

    std::vector<std::string> topics;
    topics.push_back(std::string("reconstructed_map"));

    std::vector<std::string> ref_topics;
    ref_topics.push_back(std::string("reference_map"));

    ros::Duration half_duration(0.5);

    //DEBUG
    // rosbag::Bag debug_bag;
    // debug_bag.open("/home/rpl/bagfiles/debug_reference.bag", rosbag::bagmode::Write);
    //END OF DEBUG

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

        for(rosbag::MessageInstance const ref_m: rosbag::View(ref_bag, rosbag::TopicQuery(ref_topics), start_time, end_time, true))
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

        //DEBUG -- Save ref map to see if correct

        //debug_bag.write("rec_ref_map", ref_msg->header.stamp, *ref_msg);

        //END OF DEBUG

        ufomap::Octree ref_map;
        ufomap_msgs::msgToMap(*ref_msg, ref_map);

        std::string results = EvaluationPortal::compareMaps(recon_map, ref_map);
        result_file << results;
    }

    result_file.close();
    recon_bag.close();
    ref_bag.close();

    ROS_INFO("File analysis done.");
    return true;
}
 
std::string EvaluationPortal::compareMaps(ufomap::Octree reconstruction, ufomap::Octree reference)
{   
    ufomap_math::Vector3 min_corner = Vector3(0, 0.10, 0);
    ufomap_math::Vector3 max_corner = Vector3(0.4, 0.5, 0.4);

    ufomap_geometry::AABB roi = ufomap_geometry::AABB(min_corner, max_corner);

    double true_positives = 0;
    double true_positives_recon = 0;
    double ref_occupied = 0;
    double recon_occupied = 0;
    double no_voxels = 0;
    double no_unknown = 0;

    double weight = 0;
    unsigned int depth;

    for (auto it = reference.begin_leafs_bounding(roi, true, false, false, false, 0), 
        it_end = reference.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {
        depth = it.getDepth();
        weight = std::pow(8,depth);
        ref_occupied += weight;
        
        if(reconstruction.isOccupied(it.getCenter()))
        {
            true_positives += weight;
        }
    }

    for (auto it = reconstruction.begin_leafs_bounding(roi, true, false, false, false, 0), 
        it_end = reconstruction.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {
        depth = it.getDepth();
        weight = std::pow(8,depth);
        recon_occupied += weight;

        if(reference.isOccupied(it.getCenter()))
        {
            true_positives_recon += weight;
        }

    }

    for (auto it = reconstruction.begin_leafs_bounding(roi, true, true, true, false, 0), 
        it_end = reconstruction.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {   
        depth = it.getDepth();
        weight = std::pow(8,depth);
        no_voxels += weight;
    }

    for (auto it = reconstruction.begin_leafs_bounding(roi, false, false, true, false, 0), 
        it_end = reconstruction.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {
        depth = it.getDepth();
        weight = std::pow(8,depth);
        no_unknown += weight;
    }

    /* std::cout << "tp: " << true_positives << std::endl;
    std::cout << "ref occ: " << ref_occupied << std::endl;
    std::cout << "fp: " << false_positives << std::endl;
    std::cout << "rec occ: " << recon_occupied << std::endl;
    std::cout << "unknown: " << no_unknown << std::endl;
    std::cout << "voxels: " << no_voxels << std::endl; */
    
    double recall = true_positives/ref_occupied;
    double precision = true_positives_recon/recon_occupied;
    double unknown_rate = no_unknown/no_voxels;

    std::string result = std::to_string(recall) + std::string(",") + std::to_string(precision) + std::string(",") + std::to_string(unknown_rate) + std::string("\n");
    
    return result;
} 
}