#pragma once

#include <ros/ros.h>

#include <math.h>
#include <ufomap/octree.h>
#include "ufomap/octree_base.h"

#include "ufomap/geometry/frustum.h"
#include "ufomap_msgs/GetUfomap.h"
#include "ufomap_msgs/conversions.h"
#include "ufomap/geometry/ray.h"

#include "movements/geometry_pose.h"

#include "ig_active_reconstruction/world_representation_communication_interface.hpp"

namespace ufomap_ig
{
class UFOMapIGCalculator: public ig_active_reconstruction::world_representation::CommunicationInterface
{
public:
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::ResultInformation ResultInformation;
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::IgRetrievalCommand IgRetrievalCommand;
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::ViewIgResult ViewIgResult;
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::MetricInfo MetricInfo;
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::MapMetricRetrievalCommand MapMetricRetrievalCommand;
    typedef typename ig_active_reconstruction::world_representation::CommunicationInterface::MapMetricRetrievalResultSet MapMetricRetrievalResultSet;
    
    
private:
    ros::NodeHandle nh_priv;
    ros::ServiceClient map_client_;

    
public:
    UFOMapIGCalculator(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

    ResultInformation computeViewIg(IgRetrievalCommand& command, ViewIgResult& output_ig);

    void availableIgMetrics( std::vector<MetricInfo>& available_ig_metrics );

    ResultInformation computeMapMetric(MapMetricRetrievalCommand& command, MapMetricRetrievalResultSet& output);

    void availableMapMetrics( std::vector<MetricInfo>& available_map_metrics );

private:

    bool requestMap(ufomap::Octree map);

    //IG-formulations:

    bool averageEntropyIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin, double ig_result);

    bool occlusionAwareIg(ufomap::Octree map, ufomap_geometry::AABB bb, ufomap::Point3 origin, double ig_result);

    bool unknownIg(ufomap::Octree map, ufomap_geometry::AABB bb, double ig_result);

    bool rearSideVoxelIg(ufomap::Octree, double ig_result);

    bool rearSideEntropyIg(ufomap::Octree, double ig_result);

    bool proximityCountIg(ufomap::Octree, double ig_result);

    bool areafactorIg(ufomap::Octree, double ig_result);

    bool randomIg(double ig_result);



};
}