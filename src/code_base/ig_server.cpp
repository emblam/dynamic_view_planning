#include "dynamic_view_planning/ig_server.hpp"

namespace dynamic_view_planning
{
IGCalculator::IGCalculator(ros::NodeHandle nh, ros::NodeHandle nh_priv)
    : nh_(nh)
    , nh_priv_(nh_priv_)
{
    calculate_ig_server_ = nh_.advertiseService("request_ig", &IGCalculator::computeIG, this);
    ROS_INFO("Calculator running!");
}

bool IGCalculator::computeIG(dynamic_view_planning_msgs::RequestIG::Request &req, dynamic_view_planning_msgs::RequestIG::Response &res)
{
    ufomap_math::Vector3 origin = tools::vector3FromTFmsg(req.position);

    ufomap_math::Vector3 min_corner = Vector3(-0.05, 0.05, 0);
    ufomap_math::Vector3 max_corner = Vector3(0.45, 0.55, 0.5);
    
    ufomap::Octree map;
    ufomap_msgs::msgToMap(req.map, map);

    ufomap_geometry::AABB roi = ufomap_geometry::AABB(min_corner, max_corner);

    if (req.ig_id == "average_entropy")
    {
        res.ig = IGCalculator::averageEntropyIg(map,roi,origin);
        return true;
    }
    else if(req.ig_id == "occlusion_aware")
    {   
        res.ig= IGCalculator::occlusionAwareIg(map,roi,origin);
        return true;
    }
    else if(req.ig_id == "unknown_voxels")
    {
        res.ig= IGCalculator::unknownIg(map, roi, origin);
        return true;
    }
    else if(req.ig_id == "rear_voxels")
    {
        res.ig= IGCalculator::rearSideVoxelIg(map, roi, origin);
        return true;
    }
    else if(req.ig_id == "rear_entropy")
    {
        res.ig= IGCalculator::rearSideEntropyIg(map, roi, origin);
        return true;
    }
    else if(req.ig_id == "proximity_count")
    {
        res.ig= IGCalculator::proximityCountIg(map, roi, origin);
        return true;
    }
    else if(req.ig_id == "area_factor")
    {
        res.ig= IGCalculator::areaFactorIg(map, roi, origin);
        return true;
    }
    else if(req.ig_id == "random")
    {
        res.ig= IGCalculator::randomIg();
        return true;
    }
    else
    {
        ROS_ERROR("Invalid ig_id: %s", req.ig_id);
        throw std::runtime_error("Invalid view_id.");
        return false;
    }

}

//TODO: Implement integrating method from STL Planning
double IGCalculator::averageEntropyIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin)
{
    float no_voxels;
    float accumulated_entropy;
    float temp_prob;

    for (auto it = map.begin_leafs_bounding(bb, true, true, true, false, 0), 
            it_end = map.end_leafs<ufomap_geometry::AABB>(); 
            it != it_end && ros::ok(); ++it)
    { 
        
        temp_prob = it.getProbability();
        ++no_voxels;
        accumulated_entropy += -temp_prob*log(temp_prob)-(1-temp_prob)*log(1-temp_prob);
    }

    return accumulated_entropy/no_voxels;

}

double IGCalculator::occlusionAwareIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin)
{
    double accumulated_gain;
    double voxel_entropy;
    double voxel_visibility;

    double p;

    for (auto it = map.begin_leafs_bounding(bb, true, true, true, false, 0), 
        it_end = map.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {
        std::vector<ufomap::Point3> ray;
        ufomap::Point3 end = it.getCenter();

        map.computeRay(origin, end, ray);
        
        voxel_visibility = 1;

        for (auto point = ray.begin(); point != ray.end(); point++)
        {   p = map.getNodeOccupancy(*point);
            voxel_visibility = voxel_visibility*(1-p);
        }
        p = map.getNodeOccupancy(end);
        voxel_entropy = -p*log(p)-(1-p)*log(1-p);
        accumulated_gain += voxel_visibility*voxel_entropy;
    }

    return accumulated_gain;
}

double IGCalculator::unknownIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin)
{
    double out = 0;

    return out;
}

double IGCalculator::rearSideVoxelIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin)
{
    double out = 0;

    return out;
}

double IGCalculator::rearSideEntropyIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin)
{
    double out = 0;

    return out;
}

double IGCalculator::proximityCountIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin)
{
    double out = 0;

    return out;
}

double IGCalculator::areaFactorIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin)
{
    double out = 0;

    return out;
}

double IGCalculator::randomIg()
{
    return rand() % 100;
}

}