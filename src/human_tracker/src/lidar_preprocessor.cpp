#include "human_tracker/lidar_preprocessor.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

namespace human_tracker {

CloudPtr preprocess(const CloudPtr& input,
                    const PreprocessorParams& p,
                    float robot_x,
                    float robot_y) {
    CloudPtr height_filtered(new Cloud);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(p.height_min, p.height_max);
    pass.filter(*height_filtered);

    CloudPtr radius_filtered(new Cloud);
    radius_filtered->reserve(height_filtered->size());
    const float r_min2 = p.radius_min * p.radius_min;
    const float r_max2 = p.radius_max * p.radius_max;
    for (const auto& pt : *height_filtered) {
        const float dx = pt.x - robot_x;
        const float dy = pt.y - robot_y;
        const float d2 = dx * dx + dy * dy;
        if (d2 >= r_min2 && d2 <= r_max2) {
            radius_filtered->push_back(pt);
        }
    }

    CloudPtr downsampled(new Cloud);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(radius_filtered);
    vg.setLeafSize(p.voxel_leaf, p.voxel_leaf, p.voxel_leaf);
    vg.filter(*downsampled);

    return downsampled;
}

} // namespace human_tracker
