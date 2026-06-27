#pragma once

#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace human_tracker {

constexpr int STATE_DIM = 4;   // [x, y, vx, vy]
constexpr int OBS_DIM   = 2;   // [x, y]
constexpr int PREDICT_N = 30;  // horizon steps (must match MPC N)

using StateVec = Eigen::Vector4d;
using ObsVec   = Eigen::Vector2d;
using StateCov = Eigen::Matrix4d;

enum class TrackStatus     { tentative, confirmed, lost };
enum class TrackMotionState { candidate, moving, paused };

struct KalmanParams {
    double dt{0.1};
    double q_pos{0.1};
    double q_vel{0.5};
    double r_pos{0.2};
    double gate_dist{1.0};
    int    confirm_frames{3};
    int    lost_frames{10};
    // Motion classification
    double motion_displacement_thresh{0.20}; // [m] displacement from spawn (must also exceed speed thresh)
    double motion_speed_thresh{0.25};        // [m/s] Kalman velocity estimate to classify as human
    int    static_timeout_frames{50};        // frames in candidate before discarding as static object
    int    pause_timeout_frames{300};        // frames paused before discarding (30s at 10Hz)
};

struct Prediction {
    std::array<double, PREDICT_N> x{};
    std::array<double, PREDICT_N> y{};
};

struct Track {
    uint32_t        id;
    TrackStatus     status{TrackStatus::tentative};
    TrackMotionState motion_state{TrackMotionState::candidate};
    StateVec        state{StateVec::Zero()};
    StateCov        cov{StateCov::Identity()};
    int             seen_count{0};
    int             missed_count{0};
    int             motion_frames{0};  // frames in current motion_state
    double          spawn_x{0.0};      // position at confirmation (for displacement check)
    double          spawn_y{0.0};

    Prediction predict_horizon(double dt, int N) const;
    double distance_to(double obs_x, double obs_y) const;
};

class KalmanTracker {
public:
    explicit KalmanTracker(const KalmanParams& params);

    // Update with new detections (x, y pairs in odom frame)
    void update(const std::vector<ObsVec>& detections);

    const std::vector<Track>& confirmed_tracks() const;

private:
    void predict_all();
    void associate_and_correct(const std::vector<ObsVec>& detections);
    void manage_lifecycles();

    KalmanParams           params_;
    std::vector<Track>     tracks_;
    uint32_t               next_id_{0};

    Eigen::Matrix4d F_;   // state transition
    Eigen::Matrix<double, 2, 4> H_;  // observation
    Eigen::Matrix4d Q_;   // process noise
    Eigen::Matrix2d R_;   // measurement noise
};

} // namespace human_tracker
