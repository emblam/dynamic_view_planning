#include "dynamic_view_planning/ufomap_world_representation.hpp"

#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include <future>

namespace dynamic_ufomap
{
UFOMapWorld::UFOMapWorld(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : nh_(nh)
    , nh_priv_(nh_priv)
    , map_pub_(nh_priv.advertise<ufomap_msgs::Ufomap>(
				"map", nh_priv.param("map_queue_size", 10), nh_priv.param("map_latch", false)))
    , tf_listener_(tf_buffer_)
    , map_(nh_priv.param("resolution", 0.1), nh_priv.param("depth_levels", 16),
				 !nh_priv.param("multithreaded", false))
{
    cloud_in_ = std::string("/camera_front/color/depth/points");
    cloud_sub_ = nh.subscribe(cloud_in_, nh_priv.param("cloud_in_queue_size", 10),
														&UFOMapWorld::cloudCallback, this);


	get_map_server_ = nh.advertiseService("get_map", &UFOMapWorld::exportMap, this);
	change_input_server_ = nh.advertiseService("change_camera", &UFOMapWorld::changeInput,this);

}

bool UFOMapWorld::exportMap(ufomap_msgs::GetUfomap::Request &req, ufomap_msgs::GetUfomap::Response &res)
{
	ufomap_msgs::Ufomap map_msg;
	ROS_INFO("Map requested.");

	bool success = mapToMsg(map_, map_msg);
		
	if (success)
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
	//TODO:: Add try/catch?

	cloud_in_ = req.new_camera;
	cloud_sub_ = nh_.subscribe(cloud_in_, nh_priv_.param("cloud_in_queue_size", 10),
														&UFOMapWorld::cloudCallback, this);

	return true;

}

//From ufomap_mapping -> server.cpp

void UFOMapWorld::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	try
	{
		ufomap::PointCloud cloud;

		// auto a1 and a2 are same as:
		// geometry_msgs::TransformStamped tmp_transform =
		// tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id, msg->header.stamp,
		// transform_timeout_);
		// ufomap::toUfomap(msg, cloud);

		auto a1 = std::async(std::launch::async, [this, &msg] {
			return tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id,
																				msg->header.stamp, transform_timeout_);
		});
		auto a2 =
				std::async(std::launch::async, [&msg, &cloud] { ufomap::toUfomap(msg, cloud); });

		ufomap_math::Pose6 transform = ufomap::toUfomap(a1.get().transform);
		a2.wait();
		cloud.transform(transform);
		map_.insertPointCloud(transform.translation(), cloud, 2.0);

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

	if (0 < map_pub_.getNumSubscribers() || map_pub_.isLatched())
	{
		ufomap_msgs::Ufomap msg;
		ufomap_msgs::mapToMsg(map_, msg, false);
		msg.header = header;
		map_pub_.publish(msg);
	}

}

}