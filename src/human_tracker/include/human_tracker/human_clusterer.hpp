#pragma once

#include "human_tracker/lidar_preprocessor.hpp"
#include <Eigen/Core>
#include <vector>

namespace human_tracker {

struct ClustererParams {
    float cluster_tolerance{0.3f};  // [m] max distance between cluster points
    int   min_cluster_size{10};
    int   max_cluster_size{500};
    float bb_min_xy{0.2f};   // [m]
    float bb_max_xy{0.9f};   // [m]
    float bb_min_z{0.5f};    // [m]
    float bb_max_z{2.1f};    // [m]
};

struct DetectedHuman {
    float x;
    float y;
    float z;
};

// obstacles uses nearest point to robot, not centroid
// pass nullptr for obstacles to skip collection
std::vector<DetectedHuman> extract_humans(
    const CloudPtr& cloud,
    const ClustererParams& p,
    float robot_x, float robot_y,
    float obstacle_radius_max,
    std::vector<DetectedHuman>* obstacles);

} // namespace human_tracker
