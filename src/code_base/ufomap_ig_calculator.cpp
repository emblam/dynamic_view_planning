#include "dynamic_view_planning/ufomap_ig_calculator.hpp"
#include "dynamic_view_planning/tools.hpp"

namespace ufomap_ig
{
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::ResultInformation ResultInformation;
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::IgRetrievalCommand IgRetrievalCommand;
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::ViewIgResult ViewIgResult;
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::MetricInfo MetricInfo;
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::MapMetricRetrievalCommand MapMetricRetrievalCommand;
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::MapMetricRetrievalResultSet MapMetricRetrievalResultSet;
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::IgRetrievalConfig IgRetrievalConfig;
    

UFOMapIGCalculator::UFOMapIGCalculator(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
{
    map_client_ = nh_priv.serviceClient<ufomap_msgs::GetUfomap>("get_map");
 
    
}

ResultInformation UFOMapIGCalculator::computeViewIg(IgRetrievalCommand& command, ViewIgResult& output_ig)
{

    ufomap_math::Vector3 origin = tools::poseToVector3(command.path[0]);

    IgRetrievalConfig config = command.config;
    
    std::vector<std::string> metric_names = command.metric_names;

    ufomap_math::Vector3 min_corner = Vector3(-0.05, 0.05, 0);
    ufomap_math::Vector3 max_corner = Vector3(0.45, 0.55, 0.5)

    ufomap_geometry::AABB roi = ufomap_geometry::AABB(min_corner, max_corner);


    ufomap::Octree map;
    
    if (!requestMap(map))
    {
        ROS_WARN("Could not fetch map.");
    }
    else
    {
        for (int i = 0; i < metric_names.size(); ++i)
        {
            if (metric_names[i] == std::string("AverageEntropyIg"))
            {
                if (UFOMapIGCalculator::averageEntropyIg(map, roi, output_ig[i].predicted_gain))
                {
                    output_ig[i].predicted_gain = ResultInformation::SUCCEEDED;
                }
                else
                {
                    output_ig[i].predicted_gain = ResultInformation::FAILED;
                }
                
            }
            else if (metric_names[i] == std::string("UnknownIg"))
            {
                if (UFOMapIGCalculator::unknownIg(map, roi, output_ig[i].predicted_gain))
                {
                    output_ig[i].predicted_gain = ResultInformation::SUCCEEDED;
                }
                else
                {
                    output_ig[i].predicted_gain = ResultInformation::FAILED;
                }
                
            }
            if (metric_names[i] == std::string("RandomIg"))
            {
                if (UFOMapIGCalculator::randomIg(output_ig[i].predicted_gain))
                {
                    output_ig[i].predicted_gain = ResultInformation::SUCCEEDED;
                }
                else
                {
                    output_ig[i].predicted_gain = ResultInformation::FAILED;
                }     
            }
            else
            {
                output_ig[i].predicted_gain = ResultInformation::UNKNOWN_METRIC;
            }
         }    
    }   
}

void UFOMapIGCalculator::availableIgMetrics(std::vector<MetricInfo>& available_ig_metrics)
{
    
}

ResultInformation UFOMapIGCalculator::computeMapMetric(MapMetricRetrievalCommand& command, MapMetricRetrievalResultSet& output)
{
    return ResultInformation::FAILED;
}

void UFOMapIGCalculator::availableMapMetrics( std::vector<MetricInfo>& available_map_metrics )
{
    
}

bool UFOMapIGCalculator::requestMap(ufomap::Octree map)
{
    ufomap_msgs::GetUfomap srv;

    if (map_client_.call(srv))
    {
        ROS_INFO("Map received!");
        return(msgToMap(srv.response.map, map));
    }
    else
    {
        ROS_WARN("Map not available.");
        return false;
    }
}

//TODO se till att det faktiskt är skillnad på positioner!
bool UFOMapIGCalculator::averageEntropyIg(ufomap::Octree map, ufomap_geometry::AABB bb, 
                                                ufomap::Point3 origin, double ig_result)
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

    ig_result = accumulated_entropy/no_voxels;

    return true;

}
//Should work, but is ugly and perhaps slow?
bool UFOMapIGCalculator::occlusionAwareIg(ufomap::Octree map, ufomap_geometry::AABB bb, 
                                            ufomap::Point3 origin, double ig_result)
{   
    double accumulated_gain;
    double voxel_entropy;
    double voxel_visibility;

    double p;

    for (auto it = map.begin_leafs_bounding(bb, true, true, true, false, 0), 
        it_end = map.end_leafs<ufomap_geometry::AABB>(); 
        it != it_end && ros::ok(); ++it)
    {
        ufomap::Ray ray;
        ufomap::Point3 end = it.getCenter();

        computeRay(origin, end, ray);
        
        voxel_visibility = 1;

        for (auto point = ray.begin(); point != ray.end(); point++)
        {   p = map.getNodeOccupancy(point);
            voxel_visibility = voxel_visibility*(1-p);
        }
        p = map.getNodeOccupancy(end);
        voxel_entropy = -p*log(p)-(1-p)*log(1-p);
        accumulated_gain += voxel_visibility*voxel_entropy;
    }

    ig_result = accumulated_gain;

    ig_result = 0;

    return true;

}

bool UFOMapIGCalculator::unknownIg(ufomap::Octree map, ufomap_geometry::AABB bb, double ig_result)
{
    for (auto it = map.begin_leafs_bounding(bb, false, false, true, false, 0), 
            it_end = map.end_leafs<ufomap_geometry::AABB>(); 
            it != it_end && ros::ok(); ++it)
    {
        ++ig_result;
    }

    return true;
}

bool UFOMapIGCalculator::rearSideVoxelIg(ufomap::Octree map, double ig_result)
{
    ig_result = 0;

    return true;
}

bool UFOMapIGCalculator::rearSideEntropyIg(ufomap::Octree map, double ig_result)
{
    ig_result = 0;

    return true;
}

bool UFOMapIGCalculator::proximityCountIg(ufomap::Octree map, double ig_result)
{
    ig_result = 0;

    return true;
}

bool UFOMapIGCalculator::areafactorIg(ufomap::Octree map, double ig_result)
{
    ig_result = 0;

    return true;
}

bool UFOMapIGCalculator::randomIg(double ig_result)
{
    ig_result = rand() % 100;

    return true;
}

}