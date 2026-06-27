#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace human_tracker {

struct PreprocessorParams {
    float height_min{0.3f};
    float height_max{2.0f};
    float radius_min{0.35f};    // [m] dead zone around robot chassis
    float radius_max{10.0f};
    float voxel_leaf{0.05f};
};

using Cloud    = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = Cloud::Ptr;

// cloud must be in odom frame, robot_x/y used for radius filter
CloudPtr preprocess(const CloudPtr& input,
                    const PreprocessorParams& p,
                    float robot_x,
                    float robot_y);

} // namespace human_tracker
