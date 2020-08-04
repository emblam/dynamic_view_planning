#pragma once

#include <ros/ros.h>
#include "ufomap/octree_dynamic.h"
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

namespace evaluation
{
    std::vector<ufomap::Point3> createSamplePoints(ufomap::Point3 min, ufomap::Point3 max, float resolution);

    std::vector<float> comparePoints(std::vector<ufomap::Point3> sample_points, ufomap::OctreeDynamic reference_map,
                                    ufomap::OctreeDynamic map);    

}