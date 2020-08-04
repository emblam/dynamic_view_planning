#include "dynamic_view_planning/ufomap_compare.hpp"

namespace evaluation
{
    std::vector<ufomap::Point3> createSamplePoints(ufomap::Point3 min, ufomap::Point3 max, double resolution)
    {
        std::vector<ufomap::Point3> sample_points; 

        double no_points_x = (max.x() - min.x())/resolution;
        double no_points_y = (max.y() - min.y())/resolution;
        double no_points_z = (max.z() - min.z())/resolution;

        for (int k = 0, k != no_points_x, ++k)
        {
            for (int l = 0, l != no_points_y, ++l)
            {
                for (int m = 0, l != no_points_z, ++m)
                {
                    sample_points.push_back(ufomap::Vector3(min.x() + k*resolution,
                                                            min.y() + l*resolution,
                                                            min.z() + m*resolution));
                }
            }
        }

        return sample_points;
    }

    double comparePoints(std::vector<ufomap::Point3> sample_points, ufomap::OctreeDynamic reference_map,
                                    ufomap::OctreeDynamic map)
    {   
        double correct_points = 0;
        double fraction = 0;

        for (auto it = sample_points.begin(), it != sample_points.end(); ++it)
        {
            double prob = map.getNodeOccupancy(it);
            double ref_prob = 

            if ()
            {
                ++correct_points; 
            }
        }

        fraction = correct_points/sample_points.size();

        return fraction;


    }
}