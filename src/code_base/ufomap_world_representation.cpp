#include "dynamic_view_planning/ufomap_world_representation.hpp"

#include <ros/ros.h>

#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include <future>



namespace dynamic_ufomapping
{
UFOMapWorld::UFOMapWorld(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : nh_(nh)
    , nh_priv_(nh_priv)
    , map_pub_(nh_priv.advertise<ufomap_msgs::Ufomap>(
				"ufomap_world", nh_priv.param("map_queue_size", 10), nh_priv.param("map_latch", false)))
    , tf_listener_(tf_buffer_)
    , map_(nh_priv.param("resolution", 0.005), nh_priv.param("depth_levels", 8),
				 !nh_priv.param("multithreaded", false))
	, pub_rate_(nh_priv.param("publisher_rate", 1))
	, frame_id_(nh_priv.param("frame_id", std::string("map")))
	, insert_discrete_(nh_priv.param("discrete_insertion", true))
	, insert_depth_(nh_priv.param("insert_depth", 3))
	, insert_n_(nh_priv.param("insert_n", 0))
	, max_range_(nh_priv.param("max_range", 2))
	, cloud_in_queue_size_(nh_priv.param("cloud_in_queue_size", 10))
	, map_queue_size_(nh_priv.param("map_queue_size", 10))
{	
    cloud_in_ = std::string("/camera_front/depth/color/points");

	cloud_sub_ = nh_.subscribe(cloud_in_, cloud_in_queue_size_, &UFOMapWorld::cloudCallback,this);

	if (0 < pub_rate_)
	{
		pub_timer_ = nh_priv_.createTimer(ros::Rate(pub_rate_), &UFOMapWorld::timerCallback, this);
	}

	get_map_server_ = nh_.advertiseService("get_map", &UFOMapWorld::exportMap, this);
	change_input_server_ = nh_.advertiseService("change_camera", &UFOMapWorld::changeInput,this);

	//ROS_INFO("Created object!");
}

bool UFOMapWorld::exportMap(ufomap_msgs::GetUfomap::Request &req, ufomap_msgs::GetUfomap::Response &res)
{
	ufomap_msgs::Ufomap map_msg;
	ROS_INFO("Map requested.");

	bool map_to_message = mapToMsg(map_, map_msg);
		
	if (map_to_message)
	{
		res.map = map_msg;
		ROS_INFO("Map sent.");
		return true;
	}
	else 
	{
		ROS_WARN("Map request failed.");
		return false;
	}
	
}

bool UFOMapWorld::changeInput(dynamic_view_planning::ChangeCamera::Request &req, dynamic_view_planning::ChangeCamera::Response &res)
{ 
	std::string camera_name = req.new_camera;
	cloud_in_ = "/" + camera_name + "/depth/color/points";
	
	cloud_sub_ = nh_.subscribe(cloud_in_, cloud_in_queue_size_,
														&UFOMapWorld::cloudCallback, this);

	ROS_INFO("Input topic changed.");

	return true;

}

void UFOMapWorld::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{	
	// ROS_INFO("Had a cloud_callback!");
	try
	{
		ufomap::PointCloud cloud;

		// auto a1 and a2 are same as:
		// geometry_msgs::TransformStamped tmp_transform =
		// tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id, msg->header.stamp,
		// transform_timeout_);
		// ufomap::toUfomap(msg, cloud);

 		auto a1 = std::async(std::launch::async, [this, &msg] {
			return tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id, msg->header.stamp, transform_timeout_);
		});
		auto a2 =
				std::async(std::launch::async, [&msg, &cloud] { ufomap::toUfomap(msg, cloud); });

		ufomap_math::Pose6 transform = ufomap::toUfomap(a1.get().transform);
		a2.wait(); 

		cloud.transform(transform);
		if (insert_discrete_)
		{
			map_.insertPointCloudDiscrete(transform.translation(), cloud, max_range_, insert_n_,
																		insert_depth_);
		}
		else
		{
			map_.insertPointCloud(transform.translation(), cloud, max_range_);
		}

		// ROS_INFO("Cloud inserted!");

	}
	catch (tf2::TransformException& ex)
	{
		ROS_WARN_THROTTLE(1, "%s", ex.what());
	}
}

void UFOMapWorld::timerCallback(const ros::TimerEvent& event)
{
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = frame_id_;

	//ROS_INFO("Had a timer callback.");

	if (0 < map_pub_.getNumSubscribers() || map_pub_.isLatched())
	{
		ufomap_msgs::Ufomap msg;
		ufomap_msgs::mapToMsg(map_, msg, false);
		msg.header = header;
		map_pub_.publish(msg);

		//ROS_INFO("Published on map_pub_");
	}

}

}