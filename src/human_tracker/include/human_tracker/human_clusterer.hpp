#pragma once

#include "human_tracker/lidar_preprocessor.hpp"
#include <Eigen/Core>
#include <vector>

namespace human_tracker {

struct ClustererParams {
    float cluster_tolerance{0.3f};  // [m] max distance between cluster points
    int   min_cluster_size{10};
    int   max_cluster_size{500};
    // Bounding box heuristics for human validation
    float bb_min_xy{0.2f};   // [m] min width/depth of cluster
    float bb_max_xy{0.9f};   // [m] max width/depth
    float bb_min_z{0.5f};    // [m] min height
    float bb_max_z{2.1f};    // [m] max height
};

struct DetectedHuman {
    float x;   // centroid x in odom frame [m]
    float y;   // centroid y in odom frame [m]
    float z;   // centroid z [m]
};

// Extract human candidates from preprocessed cloud.
// Returns centroids (x, y) of clusters that pass bounding-box heuristics.
std::vector<DetectedHuman> extract_humans(const CloudPtr& cloud,
                                          const ClustererParams& p);

} // namespace human_tracker
