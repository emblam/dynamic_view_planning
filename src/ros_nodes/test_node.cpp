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
#include <ufomap/code.h>
#include <ufomap/key.h>

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

    //Testing on reference map for fish_1
    rosbag::Bag test_bag;
    test_bag.open("/home/rpl/bagfiles/fish_1_random.bag");

    std::vector<std::string> topics;
    topics.push_back(std::string("reconstructed_map"));

    ufomap::Point3 origin(0.608714, 0.334438, 0.440150);
    //ufomap::Point3 origin(0.224978,-0.168973,0.458341);
    ufomap_math::Vector3 min_corner = Vector3(-0.05, 0.05, 0);
    ufomap_math::Vector3 max_corner = Vector3(0.45, 0.55, 0.5);
    
    ufomap_geometry::AABB bb = ufomap_geometry::AABB(min_corner, max_corner);


    for(rosbag::MessageInstance const m: rosbag::View(test_bag, rosbag::TopicQuery(topics)))
    {
        ufomap_msgs::Ufomap::ConstPtr map_msg = m.instantiate<ufomap_msgs::Ufomap>();

        ufomap::Octree map;
        ufomap_msgs::msgToMap(*map_msg,map);

        //START OF TEST-SCRIPT:
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
            {   
                p = map.probability(map.getNode(*point));
                voxel_visibility = voxel_visibility*(1-p);
            }
            p = map.probability(map.getNode(end));
            voxel_entropy = -p*log(p)-(1-p)*log(1-p);
            accumulated_gain += voxel_visibility*voxel_entropy;
        }

        std::cout << accumulated_gain << std::endl;
        //END OF TEST-SCRIPT
        break;

    }

    
}

/* 
double p_node = map.getNodeOccupancy(*it);
            double test_p = map.probability(map.getNode(coordinates));
 */

