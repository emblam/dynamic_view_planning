#include "dynamic_view_planning/evaluation_tools.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <fstream>

#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include <future>
namespace evaluation_tools
{


EvaluationPortal::EvaluationPortal(ros::NodeHandle nh, ros::NodeHandle nh_priv)
    : nh_(nh)
    , nh_priv_(nh_priv)
    , tf_listener_(tf_buffer_)
    , reference_map_(nh_priv.param("resolution", 0.005), nh_priv.param("depth_levels", 8),
				 !nh_priv.param("multithreaded", false))
    , experiment_name_(nh_priv.param("experiment_name", std::string("test")))
    , create_reference_(nh_priv.param("create_reference", true))
{
    data_bag_ = experiment_name_ + ".bag";
    reference_bag_ = experiment_name_ + "_reference.bag";
        
    if(create_reference_)
    {
        generateReferenceMap();
    }
}

bool EvaluationPortal::analyseFile(dynamic_view_planning::AnalyseFile::Request &req, dynamic_view_planning::AnalyseFile::Response &)
{
    std::string input_file = req.input_file;
    std::string output_file = req.output_file;

    std::ofstream result_file(output_file.c_str());

    rosbag::Bag recon_bag;
    recon_bag.open(input_file, rosbag::bagmode::Read);

    rosbag::Bag ref_bag;
    ref_bag.open(reference_bag_);

    std::vector<std::string> topics;
    topics.push_back(std::string("/reconstructed_map"));

    for(rosbag::MessageInstance const m: rosbag::View(recon_bag, rosbag::TopicQuery(topics)))
    {
        ufomap_msgs::Ufomap::ConstPtr recon_msg = m.instantiate<ufomap_msgs::Ufomap>();
        
        //TODO: Extract ref_msg from ref_bag; 
        //HOW TO DO: Extract ref_msg at timestamp? 
        ufomap_msgs::Ufomap ref_msg; 
        ufomap::Octree ref_map;
        ufomap_msgs::msgToMap(ref_msg, ref_map);


        ufomap::Octree recon_map;
        ufomap_msgs::msgToMap(*recon_msg,recon_map);

        
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

void EvaluationPortal::generateReferenceMap()
{   
    rosbag::Bag data_bag;
    data_bag.open(data_bag_, rosbag::bagmode::Read);

    rosbag::Bag reference_bag;
    reference_bag.open(reference_bag_, rosbag::bagmode::Write);

    std::vector<std::string> topics;
    topics.push_back(std::string("/camera_front/depth/color/points"));
    topics.push_back(std::string("/camera_left/depth/color/points"));
    topics.push_back(std::string("/camera_right/depth/color/points"));

    for(rosbag::MessageInstance const m: rosbag::View(data_bag, rosbag::TopicQuery(topics)))
    {
        sensor_msgs::PointCloud2::ConstPtr in_msg = m.instantiate<sensor_msgs::PointCloud2>();

        try
	    {

		    ufomap::PointCloud cloud;

    		auto a1 = std::async(std::launch::async, [this, &in_msg] {
	    		return tf_buffer_.lookupTransform("map", in_msg->header.frame_id, 
                                                    in_msg->header.stamp, transform_timeout_);
		    });
		    auto a2 =
			    	std::async(std::launch::async, [&in_msg, &cloud] { ufomap::toUfomap(in_msg, cloud); });

		    ufomap_math::Pose6 transform = ufomap::toUfomap(a1.get().transform);
		    a2.wait();
		    cloud.transform(transform);
		    reference_map_.insertPointCloudDiscrete(transform.translation(), cloud, 1, 0, 3);

	    }
	    catch (tf2::TransformException& ex)
	    {
		    ROS_WARN_THROTTLE(1, "%s", ex.what());
            continue;
	    }


        std_msgs::Header header;
	    header.stamp = in_msg->header.stamp;
	    header.frame_id = "map";

    	ufomap_msgs::Ufomap out_msg;
	    ufomap_msgs::mapToMsg(reference_map_, out_msg);
		out_msg.header = header;

        reference_bag.write("reference_map", in_msg->header.stamp, out_msg);

    }
    data_bag.close();
    reference_bag.close();
}
}