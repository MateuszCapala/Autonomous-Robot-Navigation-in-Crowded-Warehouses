#include "human_tracker/kalman_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace human_tracker {

KalmanTracker::KalmanTracker(const KalmanParams& p) : params_(p) {
    const double dt = p.dt;

    F_ = Eigen::Matrix4d::Identity();
    F_(0, 2) = dt;
    F_(1, 3) = dt;

    H_ = Eigen::Matrix<double, 2, 4>::Zero();
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;

    Q_ = Eigen::Matrix4d::Zero();
    Q_(0, 0) = p.q_pos;
    Q_(1, 1) = p.q_pos;
    Q_(2, 2) = p.q_vel;
    Q_(3, 3) = p.q_vel;

    R_ = Eigen::Matrix2d::Identity() * p.r_pos;
}

void KalmanTracker::update(const std::vector<ObsVec>& detections) {
    predict_all();
    associate_and_correct(detections);
    manage_lifecycles();
}

const std::vector<Track>& KalmanTracker::confirmed_tracks() const {
    static std::vector<Track> confirmed;
    confirmed.clear();
    for (const auto& t : tracks_) {
        if (t.status == TrackStatus::confirmed) {
            confirmed.push_back(t);
        }
    }
    return confirmed;
}

void KalmanTracker::predict_all() {
    for (auto& t : tracks_) {
        t.state = F_ * t.state;
        t.cov   = F_ * t.cov * F_.transpose() + Q_;
    }
}

void KalmanTracker::associate_and_correct(const std::vector<ObsVec>& detections) {
    std::vector<bool> det_used(detections.size(), false);

    for (auto& t : tracks_) {
        double best_dist = std::numeric_limits<double>::max();
        int    best_idx  = -1;

        for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
            if (det_used[i]) continue;
            const double d = t.distance_to(detections[i][0], detections[i][1]);
            if (d < best_dist) {
                best_dist = d;
                best_idx  = i;
            }
        }

        if (best_idx >= 0 && best_dist <= params_.gate_dist) {
            const ObsVec innov = detections[best_idx] - H_ * t.state;
            const Eigen::Matrix2d S = H_ * t.cov * H_.transpose() + R_;
            const Eigen::Matrix<double, 4, 2> K = t.cov * H_.transpose() * S.inverse();
            t.state += K * innov;
            t.cov    = (Eigen::Matrix4d::Identity() - K * H_) * t.cov;
            t.missed_count = 0;
            ++t.seen_count;
            det_used[best_idx] = true;
        } else {
            ++t.missed_count;
        }
    }

    for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
        if (det_used[i]) continue;
        Track nt;
        nt.id           = next_id_++;
        nt.state        = StateVec{detections[i][0], detections[i][1], 0.0, 0.0};
        nt.cov          = StateCov::Identity() * 0.5;
        nt.seen_count   = 1;
        nt.missed_count = 0;
        tracks_.push_back(nt);
    }
}

void KalmanTracker::manage_lifecycles() {
    for (auto& t : tracks_) {
        if (t.status == TrackStatus::tentative && t.seen_count >= params_.confirm_frames) {
            t.status = TrackStatus::confirmed;
        }
        if (t.missed_count > params_.lost_frames) {
            t.status = TrackStatus::lost;
        }
    }
    std::erase_if(tracks_, [](const Track& t) {
        return t.status == TrackStatus::lost;
    });
}

Prediction Track::predict_horizon(double dt, int N) const {
    Prediction pred;
    const double x  = state[0];
    const double y  = state[1];
    const double vx = state[2];
    const double vy = state[3];
    for (int k = 0; k < N; ++k) {
        pred.x[k] = x + (k + 1) * dt * vx;
        pred.y[k] = y + (k + 1) * dt * vy;
    }
    return pred;
}

double Track::distance_to(double obs_x, double obs_y) const {
    return std::hypot(state[0] - obs_x, state[1] - obs_y);
}

} // namespace human_tracker
