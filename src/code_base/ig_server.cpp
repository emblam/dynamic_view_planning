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
    ufomap_math::Vector3 current_pose = tools::vector3FromTFmsg(req.current_position);

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
    else if(req.ig_id == "random")
    {
        res.ig= IGCalculator::randomIg();
        return true;
    }
    else if(req.ig_id == "hidden_voxel")
    {
        res.ig=IGCalculator::hiddenVoxelIg(map, roi, origin, current_pose);
        return true;
    }
    else if(req.ig_id == "hidden_entropy")
    {
        res.ig=IGCalculator::hiddenEntropyIg(map, roi, origin, current_pose);
        return true;
    }
    else if(req.ig_id == "voxel_count")
    {
        res.ig=IGCalculator::voxelIg(map,roi,origin);
        return true;
    }
    else
    {
        ROS_ERROR("Invalid ig_id: %s", req.ig_id);
        throw std::runtime_error("Invalid view_id.");
        return false;
    }

}

double IGCalculator::averageEntropyIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin)
{
    double no_voxels = 0;
    double accumulated_entropy = 0;
    double temp_prob = 0;
    double weight = 0;
    unsigned int depth = 0;

    ufomap::Point3 end;
    ufomap::Point3 direction;

    for (auto it = map.begin_leafs_bounding(bb, true, true, true, false, 0), 
            it_end = map.end_leafs<ufomap_geometry::AABB>(); 
            it != it_end && ros::ok(); ++it)
    { 
        end = it.getCenter();
        direction = (end - origin).normalize();

        if (!(map.castRay(origin, direction, end, true, 1.0, 0)))
        {
            temp_prob = map.getNodeOccupancy(*it);
            depth = it.getDepth();
            weight = std::pow(8,depth);
            no_voxels += weight;
            accumulated_entropy += weight*(-temp_prob*log(temp_prob)-(1-temp_prob)*log(1-temp_prob));    
        }      
    }
    return accumulated_entropy/no_voxels;

}


double IGCalculator::occlusionAwareIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin)
{
    double accumulated_gain = 0;
    double voxel_entropy = 0;
    double voxel_visibility = 0;
    double no_voxels = 0;
    double weight = 0;
    unsigned int depth = 0;

    double p;

    for (auto it = map.begin_leafs_bounding(bb, true, true, true, false, 0), 
        it_end = map.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {
        std::vector<ufomap::Point3> ray;
        ufomap::Point3 end = it.getCenter();

        depth = it.getDepth();
        weight = std::pow(8,depth);
        no_voxels += weight;

        map.computeRay(origin, end, ray);
        
        voxel_visibility = 1;

        for (auto point = ray.begin(); point != ray.end(); point++)
        {   
            p = map.probability(map.getNode(*point));
            voxel_visibility = voxel_visibility*(1-p);
        }
        p = map.probability(map.getNode(end));
        voxel_entropy = -p*log(p)-(1-p)*log(1-p);
        accumulated_gain += weight*voxel_visibility*voxel_entropy;
    }

    return accumulated_gain/no_voxels;
}

double IGCalculator::unknownIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin)
{
    double unknown_voxels = 0;
    double weight = 0;
    unsigned int depth = 0;;

    ufomap::Point3 end;
    ufomap::Point3 direction;

    for (auto it = map.begin_leafs_bounding(bb, false, false, true, false, 0), 
        it_end = map.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {
        end = it.getCenter();
        direction = (end - origin).normalize();

        if (!(map.castRay(origin, direction, end, true, 1.0, 0)))
        {
            depth = it.getDepth();
            weight = std::pow(8,depth);
            unknown_voxels += weight;
        }
    }

    return unknown_voxels;
}

double IGCalculator::voxelIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin)
{
    double voxel_count = 0;
    double weight = 0;
    unsigned int depth = 0;

    ufomap::Point3 end;
    ufomap::Point3 direction;

    for (auto it = map.begin_leafs_bounding(bb, true, true, true, false, 0), 
        it_end = map.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {
        end = it.getCenter();
        direction = (end - origin).normalize();

        if (!(map.castRay(origin, direction, end, true, 1.0, 0)))
        {
            depth = it.getDepth();
            weight = std::pow(8,depth);
            voxel_count += weight;
        }
    }

    return voxel_count;
}

double IGCalculator::randomIg()
{
    return rand() % 100;
}

double IGCalculator::hiddenVoxelIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 new_view, ufomap::Point3 current_pose)
{
    double hidden_voxels = 0;
    double weight = 0;
    unsigned int depth = 0;

    ufomap::Point3 end;
    ufomap::Point3 direction;
    ufomap::Point3 direction2;
    
    if (new_view==current_pose)
    {
        return 0;
    }
    else
    {
        for (auto it = map.begin_leafs_bounding(bb, true, true, true, false, 0), 
            it_end = map.end_leafs<ufomap_geometry::AABB>(); 
            it != it_end && ros::ok(); ++it)
        {
            end = it.getCenter();
            direction = (end - new_view).normalize();
            direction2 = (end - current_pose).normalize();
        
            if (!(map.castRay(new_view, direction, end, true, 1.0, 0)) && 
                map.castRay(current_pose, direction2, end, true, 1.0, 0))
            {
                depth = it.getDepth();
                weight = std::pow(8,depth);
                hidden_voxels += weight;
            }
        }

        return hidden_voxels;
    }
}

double IGCalculator::hiddenEntropyIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 new_view, ufomap::Point3 current_pose)
{
    double hidden_entropy = 0;
    double no_voxels = 0;
    double accumulated_entropy = 0;
    double temp_prob = 0;
    double weight = 0;
    unsigned int depth = 0;

    ufomap::Point3 end;
    ufomap::Point3 direction;
    ufomap::Point3 direction2;

    if (new_view==current_pose)
    {
        return 0;
    }
    else
    {
        for (auto it = map.begin_leafs_bounding(bb, true, true, true, false, 0), 
            it_end = map.end_leafs<ufomap_geometry::AABB>(); 
            it != it_end && ros::ok(); ++it)
        {   
            end = it.getCenter();
            direction = (end - new_view).normalize();
            direction2 = (end - current_pose).normalize();

            if (!(map.castRay(new_view, direction, end, true, 1.0, 0)) && 
                map.castRay(current_pose, direction2, end, true, 1.0, 0))
            {
                temp_prob = map.getNodeOccupancy(*it);
                depth = it.getDepth();
                weight = std::pow(8,depth);
                no_voxels += weight;
                accumulated_entropy += weight*(-temp_prob*log(temp_prob)-(1-temp_prob)*log(1-temp_prob));    
            }
        }

        hidden_entropy = accumulated_entropy/no_voxels;
    
        return hidden_entropy;
    }
}

}