#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace human_tracker {

struct PreprocessorParams {
    float height_min{0.3f};     // [m] in odom frame — excludes ground
    float height_max{2.0f};     // [m] — excludes ceiling fixtures
    float radius_max{10.0f};    // [m] from robot — warehouse scale
    float voxel_leaf{0.05f};    // [m] voxel grid leaf size
};

using Cloud    = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = Cloud::Ptr;

// Filter cloud: height band + radius + voxel downsample.
// Input cloud must already be in odom frame.
// robot_x/y: current robot position in odom frame (for radius filter).
CloudPtr preprocess(const CloudPtr& input,
                    const PreprocessorParams& p,
                    float robot_x,
                    float robot_y);

} // namespace human_tracker
