#include "human_tracker/kalman_tracker.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace human_tracker;

namespace {

KalmanParams default_params() {
    return KalmanParams{
        .dt            = 0.1,
        .q_pos         = 0.1,
        .q_vel         = 0.5,
        .r_pos         = 0.2,
        .gate_dist     = 1.0,
        .confirm_frames = 3,
        .lost_frames   = 10,
    };
}

} // namespace

TEST(KalmanTracker, SpawnsTrackOnFirstDetection) {
    KalmanTracker tracker(default_params());
    tracker.update({{ObsVec{1.0, 2.0}}});
    // tentative — not confirmed yet
    EXPECT_TRUE(tracker.confirmed_tracks().empty());
}

TEST(KalmanTracker, ConfirmsTrackAfterThreeFrames) {
    KalmanTracker tracker(default_params());
    const ObsVec obs{1.0, 2.0};
    tracker.update({obs});
    tracker.update({obs});
    tracker.update({obs});
    EXPECT_EQ(tracker.confirmed_tracks().size(), 1u);
}

TEST(KalmanTracker, EstimatesVelocityFromMovingDetection) {
    KalmanTracker tracker(default_params());
    // Human walking at ~1 m/s in X direction
    const double v = 1.0;
    const double dt = 0.1;
    for (int i = 0; i < 10; ++i) {
        tracker.update({{ObsVec{i * dt * v, 0.0}}});
    }
    const auto& tracks = tracker.confirmed_tracks();
    ASSERT_FALSE(tracks.empty());
    // Velocity estimate should converge toward 1 m/s
    EXPECT_NEAR(tracks[0].state[2], v, 0.3);
    EXPECT_NEAR(tracks[0].state[3], 0.0, 0.1);
}

TEST(KalmanTracker, PredictionHorizonHasCorrectLength) {
    KalmanTracker tracker(default_params());
    const ObsVec obs{2.0, 3.0};
    for (int i = 0; i < 5; ++i) tracker.update({obs});
    const auto& tracks = tracker.confirmed_tracks();
    ASSERT_FALSE(tracks.empty());
    const auto pred = tracks[0].predict_horizon(0.1, PREDICT_N);
    EXPECT_EQ(static_cast<int>(pred.x.size()), PREDICT_N);
}

TEST(KalmanTracker, DropsLostTrack) {
    KalmanTracker tracker(default_params());
    // Confirm a track
    for (int i = 0; i < 5; ++i) tracker.update({{ObsVec{1.0, 1.0}}});
    EXPECT_EQ(tracker.confirmed_tracks().size(), 1u);
    // Stop detecting — update with empty detections until track is lost
    for (int i = 0; i < 15; ++i) tracker.update({});
    EXPECT_TRUE(tracker.confirmed_tracks().empty());
}
