#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

#include "dynamic_view_planning/view_space.hpp"
#include <tf2_ros/static_transform_broadcaster.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <ufomap/octree.h>
#include <ufomap/octree_base.h>
#include <ufomap_msgs/Ufomap.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <std_msgs/String.h>


#include "dynamic_view_planning/view_space.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <future>

#include <ufomap_msgs/GetUfomap.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv, "test_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ROS_INFO("Test node is ready");

    ufomap::Octree map;

    ros::ServiceClient get_map = nh_priv.serviceClient<ufomap_msgs::GetUfomap>("/get_map");

    ufomap_msgs::GetUfomap srv;

    ufomap::Point3 origin(0.224978,-0.168973,0.458341);

    if (get_map.call(srv))
    {
        double accumulated_gain;
        double voxel_entropy;
        double voxel_visibility;

        double p;
        
        std::cout << "Called service" << std::endl;
        ufomap_msgs::msgToMap(srv.response.map,map);

        ufomap_math::Vector3 min_corner = Vector3(-0.05, 0.05, 0);
        ufomap_math::Vector3 max_corner = Vector3(0.45, 0.55, 0.5);
    
        ufomap_geometry::AABB bb = ufomap_geometry::AABB(min_corner, max_corner);
        
        for (auto it = map.begin_leafs_bounding(bb, true, true, true, false, 0), 
            it_end = map.end_leafs<ufomap_geometry::AABB>(); 
            it != it_end && ros::ok(); ++it)
        {
            //std::cout << "Voxel iteration." << std::endl; 
            std::vector<ufomap::Point3> ray;
            ufomap::Point3 target_voxel = it.getCenter();
            p = map.getNodeOccupancy(*it);
            //std::cout << p << std::endl;

            map.computeRay(origin,target_voxel, ray);
        
            voxel_visibility = 1;

            for (std::vector<int>::size_type i = 0; 
                    i != ray.size(); i++)
            {   
                p = map.getNodeOccupancy(ray[i]);
                //std::cout << p << std::endl;
                voxel_visibility = voxel_visibility*(1-p);
            }
            p = map.getNodeOccupancy(target_voxel);
            voxel_entropy = -p*log(p)-(1-p)*log(1-p);
            accumulated_gain += voxel_visibility*voxel_entropy;

        
            //std::cout << accumulated_gain << std::endl;
        }
    }
    else
    {
        ROS_WARN("Failed to get map.");
    }

    ros::spin();
    return 0;
}
