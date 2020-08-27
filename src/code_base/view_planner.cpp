#include "dynamic_view_planning/view_planner.hpp"



namespace dynamic_view_planning
{
ViewPlanner::ViewPlanner(ros::NodeHandle nh, ros::NodeHandle nh_priv)
    : nh_(nh)
    , nh_priv_(nh_priv)
    , experiment_name_(nh_priv.param("experiment_name", std::string("test")))
    , ig_(nh_priv.param("ig_formulation", std::string("random")))
    , insert_depth_(nh_priv.param("insert_depth", 3))
	, insert_n_(nh_priv.param("insert_n", 0))
	, max_range_(nh_priv.param("max_range", 2))
    , reconstructed_map_(nh_priv.param("resolution", 0.005), nh_priv.param("depth_levels", 8),
				 !nh_priv.param("multithreaded", false))
{

    position_log_name_ = "home/rpl/results/data/" + experiment_name_ + "position_log.csv";
    ig_log_name_ = "home/rpl/results/data/" + experiment_name_ + "ig_log.csv";

    data_bag_name_ = "/home/rpl/bagfiles/" + experiment_name_ + ".bag";
    reconstructed_bag_name_ = "/home/rpl/bagfiles/" + experiment_name_ + "_" + ig_ + ".bag";

    input_topic_ = "/camera_front/depth/color/points";

    //View space
    std::string view_space_file = "/home/rpl/dvp_ws/src/dynamic_view_planning/config/" + experiment_name_ + ".txt";
    ViewSpace view_space_(view_space_file);

    //Log files
    std::ofstream position_log_file_(position_log_name_.c_str());
    std::ofstream ig_log_file_(ig_log_name_.c_str());

    //Bagfiles 
    reconstructed_bag_.open(reconstructed_bag_name_, rosbag::bagmode::Write);
    data_bag_.open(data_bag_name_, rosbag::bagmode::Read);

    start_time_ = data_bag_.get_start_time();
    end_time_ = data_bag_.get_end_time();
    request_ig_ = nh_priv.serviceClient<RequestIG>("request_ig");

    //Finding start and end time of data_bag:
    std::vector<std::string> topics;
    topics.push_back(std::string("camera_front/depth/color/points"));
    topics.push_back(std::string("camera_left/depth/color/points"));
    topics.push_back(std::string("camera_right/depth/color/points"));

    rosbag::View time_view(data_bag_, rosbag::TopicQuery(topic));

    start_time_ = time_view.getBeginTime();
    end_time_ = time_view.getEndTime();

    current_time_ = start_time_;

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
    position_log_file_.close();
    ig_log_file_.close();

    reconstructed_bag_.close();
    data_bag_.close();

    ROS_INFO("Reconstruction finished.");

}


void ViewPlanner::collectData()
{
    ros::Time start_time = current_time_;
    ros::Time end_time = start_time + time_step_;

    if (end_time > end_time_)
    {
        state_ = "shutdown";
        return;
    }
    
    current_time_ = end_time;

    std::vector<std::string> topic;
    topic.push_back(input_topic_);

    rosbag::View data_collection_view(data_bag_, rosbag::TopicQuery(topic),
                                        start_time, end_time, true);

    for(rosbag::MessageInstance const m: data_collection_view)
    {
        sensor_msgs::PointCloud2::ConstPtr in_msg = m.instantiate<sensor_msgs::PointCloud2>();

	    ufomap::PointCloud cloud;
        geometry_msgs::TransformStamped camera_position = view_space_.getViewTransform(in_msg->header.frame_id);
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
    std::string best_view;
    double ig = 0;
    for(std::vector<View>::iterator view = view_space_.view_space_.begin(); 
            it != view_space_.view_space.end(); ++it)
    {
        RequestIG srv;
        srv.request.ig_id = ig_;
        srv.request.position = view.getTransform();

        request_ig_.call(srv);

        if (srv.response.ig > ig)
        {
            best_view = view.view_id;
            ig = srv.response.ig;
        }
        
    }

    input_topic_ = "/" + best_view + "/depth/color/points";
    ig_log_file_ << ig << std::endl;
    position_log_file_ << best_view << std::endl;
    
    state_ = "collect_data"

    
}




}