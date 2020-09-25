#include "dynamic_view_planning/view_planner.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

namespace dynamic_view_planning
{
ViewPlanner::ViewPlanner(ros::NodeHandle nh, ros::NodeHandle nh_priv)
    : nh_(nh)
    , nh_priv_(nh_priv)
    , experiment_name_(nh_priv.param("experiment_name", std::string("test")))
    , ig_(nh_priv.param("ig_formulation", std::string("random")))
    , insert_depth_(nh_priv.param("insert_depth", 0))
	, insert_n_(nh_priv.param("insert_n", 0))
	, max_range_(nh_priv.param("max_range", 2))
    , reconstructed_map_(nh_priv.param("resolution", 0.005), nh_priv.param("depth_levels", 8),
				 !nh_priv.param("multithreaded", false))
    , time_step_(nh_priv.param("time_step", 1))
    , initial_view_(nh_priv.param("initial_view", std::string("camera_front")))
    , force_movement_(nh_priv.param("force_movement", false))
{
    ROS_INFO("Initiating view planning session.");
    //Generate filenames
    data_bag_name_ = "/home/rpl/bagfiles/" + experiment_name_ + ".bag";

    if (force_movement_)
    {
        position_log_name_ = "/home/rpl/results/data/" + experiment_name_ + "_" + ig_ + "_FM" + "_position_log.csv";
        ig_log_name_ = "/home/rpl/results/data/" + experiment_name_ + "_" + ig_ + "_FM" + "_ig_log.csv";
        reconstructed_bag_name_ = "/home/rpl/bagfiles/" + experiment_name_ + "_" + ig_ + "_FM" +".bag";
    }
    else
    {
        position_log_name_ = "/home/rpl/results/data/" + experiment_name_ + "_" + ig_ + "_position_log.csv";
        ig_log_name_ = "/home/rpl/results/data/" + experiment_name_ + "_" + ig_ + "_ig_log.csv";
        reconstructed_bag_name_ = "/home/rpl/bagfiles/" + experiment_name_ + "_" + ig_ + ".bag";
    }
    input_topic_ = initial_view_ + "/depth/color/points";

    //Initiate services
    request_ig_ = nh_priv.serviceClient<dynamic_view_planning_msgs::RequestIG>("/request_ig");

    //View space
    std::string view_space_file = "/home/rpl/dvp_ws/src/dynamic_view_planning/config/" + experiment_name_ + ".txt";
    view_space_ = ViewSpace(view_space_file);

    //Bagfiles 
    reconstructed_bag_.open(reconstructed_bag_name_, rosbag::bagmode::Write);
    data_bag_.open(data_bag_name_, rosbag::bagmode::Read);

    //"Clean" log files
    std::ofstream position_log_file_(position_log_name_.c_str(), std::ofstream::out | std::ofstream::trunc);
    std::ofstream ig_log_file_(ig_log_name_.c_str(), std::ofstream::out | std::ofstream::trunc);

    position_log_file_ << initial_view_ << std::endl;
    ig_log_file_ << "0,0,0" << std::endl;

    //Finding start and end time of data_bag:
    std::vector<std::string> topics;
    topics.push_back(std::string("camera_front/depth/color/points"));
    topics.push_back(std::string("camera_left/depth/color/points"));
    topics.push_back(std::string("camera_right/depth/color/points"));

    rosbag::View time_view(data_bag_, rosbag::TopicQuery(topics));

    start_time_ = time_view.getBeginTime();
    end_time_ = time_view.getEndTime();

    ROS_INFO("End time: %d", end_time_);

    current_time_ = start_time_;

    ROS_INFO("Set-up done, moving to SM.");
    state_ = "collect_data";
    // "State Machine"
    while (!(state_ == "shutdown"))
    {
        if (state_ == "collect_data")
        {
            collectData();
        }
        else if (state_ == "evaluate_views")
        {
            evaluateViews();
        }

    }

    // Shutting down.
    ROS_INFO("Shutting down.");
    position_log_file_.close();
    ig_log_file_.close();

    reconstructed_bag_.close();
    data_bag_.close();

    ROS_INFO("Reconstruction finished.");
}

void ViewPlanner::collectData()
{
    //ROS_INFO("Collecting data.");
    ros::Time start_time = current_time_;
    ros::Time end_time = start_time + time_step_;

    if (end_time > end_time_)
    {
        state_ = "shutdown";
        return;
    }
    
    current_time_ = end_time;

    ROS_INFO("Current time: %d", current_time_);

    std::vector<std::string> topic;
    topic.push_back(input_topic_);

    //std::cout << "Input topic: " << input_topic_ << std::endl;

    rosbag::View data_collection_view(data_bag_, rosbag::TopicQuery(topic),
                                        start_time, end_time, true);

    for(rosbag::MessageInstance const m: data_collection_view)
    {
        sensor_msgs::PointCloud2::ConstPtr in_msg = m.instantiate<sensor_msgs::PointCloud2>();

	    ufomap::PointCloud cloud;
        geometry_msgs::TransformStamped camera_position = view_space_.getViewTransform(in_msg->header.frame_id);
        current_position_frame_ = in_msg->header.frame_id;
        ufomap_math::Pose6 transform = ufomap::toUfomap(camera_position.transform);
        ufomap::toUfomap(in_msg, cloud);

	    cloud.transform(transform);
	    reconstructed_map_.insertPointCloudDiscrete(transform.translation(), cloud, max_range_, insert_n_, insert_depth_);

        std_msgs::Header header;
	    header.stamp = in_msg->header.stamp;
	    header.frame_id = "map";

    	ufomap_msgs::Ufomap out_msg;
	    ufomap_msgs::mapToMsg(reconstructed_map_, out_msg);
		out_msg.header = header;

        reconstructed_bag_.write("reconstructed_map", in_msg->header.stamp, out_msg);
    }

    state_ = "evaluate_views";
}

void ViewPlanner::evaluateViews()
{
    //ROS_INFO("Evaluating views.");
    std::string best_view;
    double ig = 0;

    std::ofstream position_log_file_(position_log_name_.c_str(), std::ofstream::app);
    std::ofstream ig_log_file_(ig_log_name_.c_str(), std::ofstream::app);

    for(std::vector<int>::size_type i = 0; 
            i != view_space_.view_space_.size(); i++)
    {
        geometry_msgs::TransformStamped current_position = view_space_.getViewTransform(current_position_frame_);

        if (force_movement_)
        {
            View current_view = view_space_.getViewFromFrame(current_position_frame_);
            
            if (view_space_.view_space_[i]==current_view)
            {
                if (i == 0)
                {
                    ig_log_file_ << "0";
                }
                else
                {
                    ig_log_file_ << ",0";
                }
                continue;
            }
        }

        dynamic_view_planning_msgs::RequestIG srv;
        ufomap_msgs::Ufomap req_msg; 
        ufomap_msgs::mapToMsg(reconstructed_map_, req_msg);
        
        srv.request.map = req_msg;
        srv.request.ig_id = ig_;
        srv.request.position = view_space_.view_space_[i].getTransform();
        srv.request.current_position = current_position;

        if (request_ig_.call(srv))
        {
            //std::cout << "ig " << srv.response.ig << std::endl;
            if (i == 0)
            {
                ig_log_file_ << std::to_string(srv.response.ig);
            }
            else
            {
                ig_log_file_ << "," << std::to_string(srv.response.ig);
            }
            if (srv.response.ig > ig)
            {
                best_view = view_space_.view_space_[i].view_id;
                ig = srv.response.ig;
            }
        }
        else
        {
            std::cout << "Bad request";
        }        
    }

    input_topic_ = best_view + "/depth/color/points";
    ig_log_file_ << "\n";
    position_log_file_ << best_view << "\n";
    
    state_ = "collect_data";
}
}