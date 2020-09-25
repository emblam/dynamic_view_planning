#pragma once

#include <fstream>
#include <exception>

#include <ros/ros.h>

#include "ufomap/types.h"

#include "geometry_msgs/TransformStamped.h"


namespace dynamic_view_planning
{
class View
{
public:

    float x;
    float y;
    float z;
    
    float qx;
    float qy;
    float qz;
    float qw;

    //frame_id = tf frame, view_id = front/left/right 
    std::string frame_id;
    std::string view_id;

public: 

    View()
    {
        x = 0;
        y = 0;
        z = 0;

        qx = 0;
        qy = 0;
        qz = 0;
        qw = 1;

        frame_id = "empty_view";
    }

    ufomap::Point3 getOrigin()
    {
        ufomap::Point3 origin(x,y,z);

        return origin;
    }

    geometry_msgs::TransformStamped getTransform()
    {
        geometry_msgs::TransformStamped tf;

        tf.transform.translation.x = x;  
        tf.transform.translation.y = y;
        tf.transform.translation.z = z;

        tf.transform.rotation.x = qx;
        tf.transform.rotation.y = qy;
        tf.transform.rotation.z = qz;
        tf.transform.rotation.w = qw;

        return tf;
    }

    bool operator==(const View& view) const
    {
        return x == view.x && y == view.y && z == view.z && 
               qx == view.qx && qy == view.qy && qz == view.qz && qw == view.qw; 
    }

};

class ViewSpace
{
public:

    std::vector<View> view_space_;

public: 

    ViewSpace() = default;

    ViewSpace(const std::string& file_name)
    {
        std::ifstream in(file_name, std::ifstream::in);

        unsigned int nr_of_views;

        in>>nr_of_views;

        for(unsigned int i=0; i<nr_of_views; ++i)
        {
            View new_view;

            in >> new_view.view_id >> new_view.frame_id >> new_view.x >> new_view.y >> new_view.z >> new_view.qx >> new_view.qy >> new_view.qz >> new_view.qw;
            
            view_space_.push_back(new_view);
        }

        ROS_INFO("Viewspace created.");
    }

    const View& getView(int i) const
    {
        return view_space_[i];
    }

    const View& getView(const std::string& view_id) const
    {

        auto it = find_if(view_space_.begin(), view_space_.end(), [&view_id](const View& view) {return view.view_id == view_id;});

        if(it != view_space_.end())
        {
            auto index = std::distance(view_space_.begin(), it);

            return getView(index);
        }
        else
        {
            ROS_ERROR("Invalid view-id: %s", view_id);

            throw std::runtime_error("Invalid view_id.");

        }
    }

    const View& getViewFromFrame(const std::string& frame_id) const
    {

        auto it = find_if(view_space_.begin(), view_space_.end(), [&frame_id](const View& view) {return view.frame_id == frame_id;});

        if(it != view_space_.end())
        {
            auto index = std::distance(view_space_.begin(), it);

            return getView(index);
        }
        else
        {
            std::cout << frame_id << std::endl;
            ROS_ERROR("Invalid frame-id:");
            throw std::runtime_error("Invalid frame_id.");

        }
    }

    ufomap::Point3 getViewOrigin(const std::string& frame_id) const
    {
        View view = getViewFromFrame(frame_id);

        ufomap::Point3 origin = view.getOrigin();

        return origin;
    }

    geometry_msgs::TransformStamped getViewTransform(const std::string& frame_id) const
    {   

        View view = getViewFromFrame(frame_id);
        

        geometry_msgs::TransformStamped tf = view.getTransform();

        return tf;
    }
};

}