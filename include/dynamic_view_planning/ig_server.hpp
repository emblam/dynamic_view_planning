#pragma once

#include <ros/ros.h>

#include <ufomap/octree.h>
#include <ufomap_msgs/conversions.h>

#include "dynamic_view_planning_msgs/RequestIG.h"
#include "dynamic_view_planning/tools.hpp"



namespace dynamic_view_planning
{
class IGCalculator
{
public: 

    IGCalculator(ros::NodeHandle nh, ros::NodeHandle nh_priv);

private:

    bool computeIG(dynamic_view_planning_msgs::RequestIG::Request &req, dynamic_view_planning_msgs::RequestIG::Response &res);

    double averageEntropyIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin);
    double occlusionAwareIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin);   
    double unknownIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin);
    double rearSideVoxelIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin);
    double rearSideEntropyIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin);
    double proximityCountIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin);
    double areaFactorIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin);
    double randomIg();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    ros::ServiceServer calculate_ig_server_;





};
}