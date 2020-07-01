#pragma once

#include <Eigen/Geometry>
#include "movements/geometry_pose.h"

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

    ig_active_reconstruction::Views::View viewFromTFtopic(std::string tf_name)
    {
        ig_active_reconstruction::Views::View view;

        //Smart matchningsfunktion här!

        return view;
    }

    std::string viewToTFtopic(ig_active_reconstruction::Views::View view)
    {
        //En till smart matchningsfunktion! 
        
        return tf_name;
    }
}