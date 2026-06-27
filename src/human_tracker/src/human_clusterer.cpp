#include "human_tracker/human_clusterer.hpp"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <algorithm>
#include <limits>

namespace human_tracker {

std::vector<DetectedHuman> extract_humans(
    const CloudPtr& cloud,
    const ClustererParams& p,
    float robot_x, float robot_y,
    float obstacle_radius_max,
    std::vector<DetectedHuman>* obstacles)
{
    if (cloud->empty()) return {};

    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    ece.setClusterTolerance(p.cluster_tolerance);
    ece.setMinClusterSize(p.min_cluster_size);
    ece.setMaxClusterSize(p.max_cluster_size);
    ece.setSearchMethod(tree);
    ece.setInputCloud(cloud);
    ece.extract(cluster_indices);

    std::vector<DetectedHuman> humans;
    humans.reserve(cluster_indices.size());
    const float obs_r2 = obstacle_radius_max * obstacle_radius_max;

    for (const auto& indices : cluster_indices) {
        float x_min = std::numeric_limits<float>::max();
        float x_max = std::numeric_limits<float>::lowest();
        float y_min = std::numeric_limits<float>::max();
        float y_max = std::numeric_limits<float>::lowest();
        float z_min = std::numeric_limits<float>::max();
        float z_max = std::numeric_limits<float>::lowest();
        float cx = 0.f, cy = 0.f, cz = 0.f;

        // Nearest point to robot (for obstacle position)
        float nearest_x = 0.f, nearest_y = 0.f, nearest_z = 0.f;
        float nearest_d2 = std::numeric_limits<float>::max();

        for (int idx : indices.indices) {
            const auto& pt = (*cloud)[idx];
            cx += pt.x; cy += pt.y; cz += pt.z;
            x_min = std::min(x_min, pt.x); x_max = std::max(x_max, pt.x);
            y_min = std::min(y_min, pt.y); y_max = std::max(y_max, pt.y);
            z_min = std::min(z_min, pt.z); z_max = std::max(z_max, pt.z);

            if (obstacles) {
                const float ddx = pt.x - robot_x;
                const float ddy = pt.y - robot_y;
                const float d2  = ddx * ddx + ddy * ddy;
                if (d2 < nearest_d2) {
                    nearest_d2 = d2; nearest_x = pt.x; nearest_y = pt.y; nearest_z = pt.z;
                }
            }
        }

        const float n  = static_cast<float>(indices.indices.size());
        const float dx = x_max - x_min;
        const float dy = y_max - y_min;
        const float dz = z_max - z_min;

        const bool is_human =
            dx >= p.bb_min_xy && dx <= p.bb_max_xy &&
            dy >= p.bb_min_xy && dy <= p.bb_max_xy &&
            dz >= p.bb_min_z  && dz <= p.bb_max_z;

        if (is_human) {
            humans.push_back({cx / n, cy / n, cz / n});
        } else if (obstacles && nearest_d2 <= obs_r2) {
            // Use nearest point so the robot keeps distance from the surface
            obstacles->push_back({nearest_x, nearest_y, nearest_z});
        }
    }

    return humans;
}

} // namespace human_tracker
