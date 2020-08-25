#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

#include "dynamic_view_planning/view_space.hpp"
#include <tf2_ros/static_transform_broadcaster.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <ufomap/octree.h>
#include <ufomap_msgs/Ufomap.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <std_msgs/String.h>

#include "dynamic_view_planning/view_space.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <future>

int main(int argc, char** argv)
{
    ros::init(argc,argv, "test_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    std::string experiment_name = "fish_1";

    ROS_INFO("Test node is ready");

    double resolution = 0.005;
    int depth_levels = 16;
    bool multithreaded = false;
    float max_range = 1.0;
   	int insert_depth = 3;
	  int insert_n = 0;

    ufomap::Octree reconstructed_map = ufomap::Octree(resolution, depth_levels, multithreaded);

    std::string raw_data_bag_name = "/home/rpl/bagfiles/" + experiment_name + ".bag";
    std::string rec_bag_name = "/home/rpl/bagfiles/" + experiment_name + "_front.bag";
    std::string view_space_file = "/home/rpl/dvp_ws/src/dynamic_view_planning/config/" + experiment_name + ".txt";

    dynamic_view_planning::ViewSpace view_space(view_space_file);

    rosbag::Bag raw_data_bag;
    raw_data_bag.open(raw_data_bag_name, rosbag::bagmode::Read);

    rosbag::Bag rec_bag;
    rec_bag.open(rec_bag_name, rosbag::bagmode::Write);

    std::vector<std::string> topics;
    topics.push_back(std::string("camera_front/depth/color/points"));

    ROS_INFO("Generating front map...");

    for(rosbag::MessageInstance const m: rosbag::View(raw_data_bag, rosbag::TopicQuery(topics)))
    {
        sensor_msgs::PointCloud2::ConstPtr in_msg = m.instantiate<sensor_msgs::PointCloud2>();

	    ufomap::PointCloud cloud;
        geometry_msgs::TransformStamped camera_position = view_space.getViewTransform(in_msg->header.frame_id);
        ufomap_math::Pose6 transform = ufomap::toUfomap(camera_position.transform);
        ufomap::toUfomap(in_msg, cloud);

	    cloud.transform(transform);
	    reconstructed_map.insertPointCloudDiscrete(transform.translation(), cloud, max_range, insert_n, insert_depth);

        std_msgs::Header header;
	    header.stamp = in_msg->header.stamp;
	    header.frame_id = "map";

      	ufomap_msgs::Ufomap out_msg;
	    ufomap_msgs::mapToMsg(reconstructed_map, out_msg);
		out_msg.header = header;

        rec_bag.write("reconstructed_map", in_msg->header.stamp, out_msg);
    }

    raw_data_bag.close();
    rec_bag.close();

    ROS_INFO("Reconstructed map generated.");

    ros::spin();
    return 0;
}
