#pragma once

#include <Eigen/Geometry>
#include "movements/geometry_pose.h"
#include "ufomap/math/vector3.h"
#include "ig_active_reconstruction/view.hpp"
#include "geometry_msgs/TransformStamped.h"

namespace tools
{
    ufomap_math::Vector3 poseToVector3(movements::Pose in_pose)
    {   
        float x = in_pose.position.x();
        float y = in_pose.position.y();
        float z = in_pose.position.z();

        ufomap_math::Vector3 out_vector3 = ufomap_math::Vector3(x,y,z);
        return out_vector3;
    }

    ig_active_reconstruction::views::View viewFromTFtopic(std::string camera_name)
    {
        ig_active_reconstruction::views::View view;

        //Smart matchningsfunktion här!

        return view;
    }

    std::string viewToCameraName(ig_active_reconstruction::views::View view)
    {
        std::string camera_name;

        //En till smart matchningsfunktion!

        return camera_name;
    }

    std::string convertToString(geometry_msgs::TransformStamped tf_in, std::string camera_name)
    {
        std::string string_out;

        std::string tf_x = std::to_string(tf_in.transform.translation.x);
        std::string tf_y = std::to_string(tf_in.transform.translation.y);
        std::string tf_z = std::to_string(tf_in.transform.translation.z);
        
        std::string tf_qx = std::to_string(tf_in.transform.rotation.x);
        std::string tf_qy = std::to_string(tf_in.transform.rotation.x);
        std::string tf_qz = std::to_string(tf_in.transform.rotation.x);
        std::string tf_qw = std::to_string(tf_in.transform.rotation.x);

        string_out = tf_x + " " + tf_y + " " + tf_z + " " + tf_qx + " " + tf_qy + " " + tf_qz + " " + tf_qw + " " + camera_name + "\n";

        return string_out;
    }
}