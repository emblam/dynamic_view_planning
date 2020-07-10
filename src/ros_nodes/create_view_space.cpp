#include <ros/ros.h>
#include <fstream>
#include "dynamic_view_planning/tools.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "create_view_space_node");
    ros::NodeHandle nh;

    geometry_msgs::TransformStamped tf_front;
    geometry_msgs::TransformStamped tf_left;
    geometry_msgs::TransformStamped tf_right;

    std::string file_in;
    nh.getParam(std::string("/file_in"), file_in);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ROS_WARN_STREAM(file_in);

    bool front = false;
    bool left = false;
    bool right = false;
    while (!front)
    {
        
        try
        {
            if(tfBuffer.canTransform("map", "camera_front_link", ros::Time(0), ros::Duration(60)))
            {
                tf_front = tfBuffer.lookupTransform("map", "camera_front_link", ros::Time(0));
                front = true;
            }
            
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            front = false;
        }
    }

    while(!left)
    {
        try
        {
            if(tfBuffer.canTransform("map", "camera_left_link", ros::Time(0), ros::Duration(60)))
            {
                tf_left = tfBuffer.lookupTransform("map", "camera_left_link",ros::Time(0));
                left = true;
            }
            
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            left = false;
        }
    }

    while (!right)
    {
        try
        {
            if(tfBuffer.canTransform("map", "camera_right_link", ros::Time(0), ros::Duration(60)))
            {
                tf_right = tfBuffer.lookupTransform("map", "camera_right_link",ros::Time(0));
                right = true;
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            right = false;
        }
    }

    std::string front_view = tools::convertToString(tf_front, "camera_front");
    std::string left_view = tools::convertToString(tf_left, "camera_left");
    std::string right_view = tools::convertToString(tf_right, "camera_right");

    std::ofstream myfile(file_in.c_str());
    myfile << "3 \n";
    myfile << front_view;
    myfile << left_view;
    myfile << right_view;

    myfile.close();

    ROS_WARN("A view space file has been created: %s", file_in.c_str());
    
    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    return 0;

}