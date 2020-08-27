#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <ufomap/octree.h>
#include <ufomap_msgs/Ufomap.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include "dynamic_view_planning/view_space.hpp"
#include <future>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"generate_reference_map_node");
    ros::NodeHandle nh;

    std::string experiment_name;
    double resolution;
    int depth_levels;
    bool multithreaded;
    float max_range;
   	int insert_depth;
	int insert_n;
    
    nh.getParam(std::string("/experiment_name"), experiment_name);
    nh.getParam(std::string("/resolution"), resolution);
    nh.getParam(std::string("/depth_levels"), depth_levels);
    nh.getParam(std::string("/multithreaded"), multithreaded);
    nh.getParam(std::string("/max_range"), max_range);
    nh.getParam(std::string("/insert_depth"), insert_depth);
    nh.getParam(std::string("/insert_n"), insert_n);

    ufomap::Octree reference_map = ufomap::Octree(resolution, depth_levels, multithreaded);

    std::string raw_data_bag_name = "/home/rpl/bagfiles/" + experiment_name + ".bag";
    std::string reference_bag_name = "/home/rpl/bagfiles/" + experiment_name + "_reference.bag";
    std::string view_space_file = "/home/rpl/dvp_ws/src/dynamic_view_planning/config/" + experiment_name + ".txt";

    dynamic_view_planning::ViewSpace view_space(view_space_file);

    rosbag::Bag raw_data_bag;
    raw_data_bag.open(raw_data_bag_name, rosbag::bagmode::Read);

    rosbag::Bag reference_bag;
    reference_bag.open(reference_bag_name, rosbag::bagmode::Write);

    std::vector<std::string> topics;
    topics.push_back(std::string("camera_front/depth/color/points"));
    topics.push_back(std::string("camera_left/depth/color/points"));
    topics.push_back(std::string("camera_right/depth/color/points"));

    ROS_INFO("Generating reference map...");

    for(rosbag::MessageInstance const m: rosbag::View(raw_data_bag, rosbag::TopicQuery(topics)))
    {
        sensor_msgs::PointCloud2::ConstPtr in_msg = m.instantiate<sensor_msgs::PointCloud2>();

	    ufomap::PointCloud cloud;
        geometry_msgs::TransformStamped camera_position = view_space.getViewTransform(in_msg->header.frame_id);
        ufomap_math::Pose6 transform = ufomap::toUfomap(camera_position.transform);
        ufomap::toUfomap(in_msg, cloud);

	    cloud.transform(transform);
	    reference_map.insertPointCloudDiscrete(transform.translation(), cloud, max_range, insert_n, insert_depth);

        std_msgs::Header header;
	    header.stamp = in_msg->header.stamp;
	    header.frame_id = "map";

    	ufomap_msgs::Ufomap out_msg;
	    ufomap_msgs::mapToMsg(reference_map, out_msg);
		out_msg.header = header;

        reference_bag.write("reference_map", in_msg->header.stamp, out_msg);

        //ROS_INFO("Message added.");
    }

    raw_data_bag.close();
    reference_bag.close();

    ROS_INFO("Reference map generated.");
}