#pragma once

#include <Eigen/Geometry>
#include "ufomap/math/vector3.h"
#include "geometry_msgs/TransformStamped.h"

namespace tools
{
    inline ufomap_math::Vector3 vector3FromTFmsg(geometry_msgs::TransformStamped tf)
    {
        ufomap_math::Vector3 out_vec(tf.transform.translation.x, 
                                        tf.transform.translation.y,
                                        tf.transform.translation.z);

        return out_vec;
    }
    inline std::string convertToString(geometry_msgs::TransformStamped tf_in, std::string camera_name)
    {
        std::string string_out;

        std::string frame_id = tf_in.child_frame_id;

        std::string tf_x = std::to_string(tf_in.transform.translation.x);
        std::string tf_y = std::to_string(tf_in.transform.translation.y);
        std::string tf_z = std::to_string(tf_in.transform.translation.z);
        
        std::string tf_qx = std::to_string(tf_in.transform.rotation.x);
        std::string tf_qy = std::to_string(tf_in.transform.rotation.y);
        std::string tf_qz = std::to_string(tf_in.transform.rotation.z);
        std::string tf_qw = std::to_string(tf_in.transform.rotation.w);

        string_out = camera_name + " " + frame_id + " " + tf_x + " " + tf_y + " " + tf_z + " " + tf_qx + " " + tf_qy + " " + tf_qz + " " + tf_qw + "\n";

        return string_out;
    }

}