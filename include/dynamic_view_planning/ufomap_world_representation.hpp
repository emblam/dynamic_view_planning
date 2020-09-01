#pragma once
#include <ros/ros.h>

//#include "ufomap/octree_dynamic.h"
#include "ufomap/octree.h"

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <ufomap_msgs/GetUfomap.h>
#include "dynamic_view_planning_msgs/ChangeCamera.h"

#include "dynamic_view_planning/view_space.hpp"


namespace dynamic_ufomapping
{
class UFOMapWorld
{
public:

    UFOMapWorld(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void timerCallback(const ros::TimerEvent& event);

    bool exportMap(ufomap_msgs::GetUfomap::Request &req, ufomap_msgs::GetUfomap::Response &res);

	bool changeInput(dynamic_view_planning_msgs::ChangeCamera::Request &req, dynamic_view_planning_msgs::ChangeCamera::Response &res);

private: 

    ros::NodeHandle& nh_;
    ros::NodeHandle& nh_priv_;

    ros::Subscriber cloud_sub_;

    ros::Publisher map_pub_;

    ros::Timer pub_timer_;

    ros::ServiceServer get_map_server_;
    ros::ServiceServer change_input_server_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Duration transform_timeout_ = ros::Duration(30);

    std::string cloud_in_;
    std::string frame_id_;

    //ufomap::OctreeDynamic map_;
    ufomap::Octree map_;

    float max_range_;

    float pub_rate_;

    bool insert_discrete_;
	unsigned int insert_depth_;
	unsigned int insert_n_;

    unsigned int cloud_in_queue_size_;
    unsigned int map_queue_size_;

};
}