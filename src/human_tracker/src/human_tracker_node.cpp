#include "human_tracker/human_clusterer.hpp"
#include "human_tracker/kalman_tracker.hpp"
#include "human_tracker/lidar_preprocessor.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <social_navigation_msgs/msg/human_array.hpp>
#include <social_navigation_msgs/msg/human_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>

namespace human_tracker {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
constexpr int    M_MAX = 5;
constexpr int    N     = PREDICT_N;
constexpr double DT    = 0.1;

class HumanTrackerNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit HumanTrackerNode(const rclcpp::NodeOptions& opts)
        : rclcpp_lifecycle::LifecycleNode("human_tracker", opts) {
        declare_parameter<double>("height_min",           0.3);
        declare_parameter<double>("height_max",           2.0);
        declare_parameter<double>("radius_min",           0.35);
        declare_parameter<double>("radius_max",          10.0);
        declare_parameter<double>("voxel_leaf",           0.05);
        declare_parameter<double>("obstacle_radius_max",  4.0);
        declare_parameter<double>("cluster_tolerance", 0.3);
        declare_parameter<int>   ("min_cluster_size",  10);
        declare_parameter<int>   ("max_cluster_size",  500);
        declare_parameter<double>("gate_distance",                1.0);
        declare_parameter<int>   ("confirm_frames",             3);
        declare_parameter<int>   ("lost_frames",                10);
        declare_parameter<double>("motion_displacement_thresh", 0.20);
        declare_parameter<double>("motion_speed_thresh",        0.25);
        declare_parameter<int>   ("static_timeout_frames",      50);
        declare_parameter<int>   ("pause_timeout_frames",       300);
        declare_parameter<std::string>("lidar_topic",  "/front_3d_lidar/lidar_points");
        declare_parameter<std::string>("odom_topic",   "/chassis/odom");
        declare_parameter<std::string>("odom_frame",   "odom");
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
        PreprocessorParams pp;
        pp.height_min = static_cast<float>(get_parameter("height_min").as_double());
        pp.height_max = static_cast<float>(get_parameter("height_max").as_double());
        pp.radius_min = static_cast<float>(get_parameter("radius_min").as_double());
        pp.radius_max = static_cast<float>(get_parameter("radius_max").as_double());
        pp.voxel_leaf = static_cast<float>(get_parameter("voxel_leaf").as_double());
        preproc_params_ = pp;
        obstacle_radius_max_ = static_cast<float>(
            get_parameter("obstacle_radius_max").as_double());

        ClustererParams cp;
        cp.cluster_tolerance = static_cast<float>(get_parameter("cluster_tolerance").as_double());
        cp.min_cluster_size  = get_parameter("min_cluster_size").as_int();
        cp.max_cluster_size  = get_parameter("max_cluster_size").as_int();
        cluster_params_ = cp;

        KalmanParams kp;
        kp.dt                          = DT;
        kp.gate_dist                   = get_parameter("gate_distance").as_double();
        kp.confirm_frames              = get_parameter("confirm_frames").as_int();
        kp.lost_frames                 = get_parameter("lost_frames").as_int();
        kp.motion_displacement_thresh  = get_parameter("motion_displacement_thresh").as_double();
        kp.motion_speed_thresh         = get_parameter("motion_speed_thresh").as_double();
        kp.static_timeout_frames       = get_parameter("static_timeout_frames").as_int();
        kp.pause_timeout_frames        = get_parameter("pause_timeout_frames").as_int();
        tracker_ = std::make_unique<KalmanTracker>(kp);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        odom_frame_  = get_parameter("odom_frame").as_string();

        pub_humans_   = create_publisher<social_navigation_msgs::msg::HumanArray>(
            "/humans/tracked", rclcpp::QoS(5));
        pub_markers_  = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/humans/markers", rclcpp::QoS(5));
        pub_clusters_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/humans/debug/clusters", rclcpp::QoS(2));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
        pub_humans_->on_activate();
        pub_markers_->on_activate();
        pub_clusters_->on_activate();

        const auto lidar_topic = get_parameter("lidar_topic").as_string();
        const auto odom_topic  = get_parameter("odom_topic").as_string();

        sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { lidar_cb(msg); });

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                robot_x_ = static_cast<float>(msg->pose.pose.position.x);
                robot_y_ = static_cast<float>(msg->pose.pose.position.y);
            });

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
        sub_lidar_.reset();
        sub_odom_.reset();
        pub_humans_->on_deactivate();
        pub_markers_->on_deactivate();
        pub_clusters_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
        tracker_.reset();
        tf_listener_.reset();
        tf_buffer_.reset();
        pub_humans_.reset();
        pub_markers_.reset();
        pub_clusters_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
        return CallbackReturn::SUCCESS;
    }

private:
    void lidar_cb(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
        sensor_msgs::msg::PointCloud2 cloud_odom;
        try {
            const auto tf = tf_buffer_->lookupTransform(
                odom_frame_, msg->header.frame_id,
                msg->header.stamp, rclcpp::Duration::from_seconds(0.05));
            tf2::doTransform(*msg, cloud_odom, tf);
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN(get_logger(), "TF: %s", e.what());
            return;
        }

        CloudPtr pcl_cloud(new Cloud);
        pcl::fromROSMsg(cloud_odom, *pcl_cloud);

        const auto filtered = preprocess(pcl_cloud, preproc_params_, robot_x_, robot_y_);

        if (pub_clusters_->get_subscription_count() > 0) {
            sensor_msgs::msg::PointCloud2 debug_msg;
            pcl::toROSMsg(*filtered, debug_msg);
            debug_msg.header.frame_id = odom_frame_;
            debug_msg.header.stamp    = msg->header.stamp;
            pub_clusters_->publish(debug_msg);
        }

        latest_obstacles_.clear();
        const auto detections = extract_humans(
            filtered, cluster_params_,
            robot_x_, robot_y_,
            obstacle_radius_max_,
            &latest_obstacles_);
        std::vector<ObsVec> obs;
        obs.reserve(detections.size());
        for (const auto& d : detections) {
            obs.emplace_back(ObsVec{d.x, d.y});
        }

        tracker_->update(obs);

        publish_humans(msg->header.stamp);
        publish_markers(msg->header.stamp);
    }

    void publish_humans(const rclcpp::Time& stamp) {
        social_navigation_msgs::msg::HumanArray arr;
        arr.header.stamp    = stamp;
        arr.header.frame_id = odom_frame_;

        const auto& tracks = tracker_->confirmed_tracks();

        std::vector<const Track*> sorted;
        sorted.reserve(tracks.size());
        for (const auto& t : tracks) sorted.push_back(&t);
        std::ranges::sort(sorted, [this](const Track* a, const Track* b) {
            return a->distance_to(robot_x_, robot_y_) < b->distance_to(robot_x_, robot_y_);
        });

        arr.num_confirmed = static_cast<uint32_t>(sorted.size());

        std::vector<DetectedHuman> obs_sorted = latest_obstacles_;
        std::ranges::sort(obs_sorted, [this](const DetectedHuman& a, const DetectedHuman& b) {
            const float da = (a.x - robot_x_) * (a.x - robot_x_) + (a.y - robot_y_) * (a.y - robot_y_);
            const float db = (b.x - robot_x_) * (b.x - robot_x_) + (b.y - robot_y_) * (b.y - robot_y_);
            return da < db;
        });

        int obs_idx = 0;
        for (int i = 0; i < M_MAX; ++i) {
            social_navigation_msgs::msg::HumanState hs;
            hs.header = arr.header;

            if (i < static_cast<int>(sorted.size())) {
                const Track& t = *sorted[i];
                const auto pred = t.predict_horizon(DT, N);
                hs.id                = t.id;
                hs.x                 = t.state[0];
                hs.y                 = t.state[1];
                hs.vx                = t.state[2];
                hs.vy                = t.state[3];
                hs.distance_to_robot = t.distance_to(robot_x_, robot_y_);
                hs.pred_x.assign(pred.x.begin(), pred.x.end());
                hs.pred_y.assign(pred.y.begin(), pred.y.end());
            } else if (obs_idx < static_cast<int>(obs_sorted.size())) {
                const DetectedHuman& ob = obs_sorted[obs_idx++];
                hs.id  = UINT32_MAX - 1 - static_cast<uint32_t>(obs_idx);
                hs.x   = ob.x; hs.y = ob.y;
                hs.vx  = 0.0;  hs.vy = 0.0;
                hs.distance_to_robot = std::hypot(ob.x - robot_x_, ob.y - robot_y_);
                hs.pred_x.assign(N, ob.x);
                hs.pred_y.assign(N, ob.y);
            } else {
                hs.id = UINT32_MAX;
                hs.x  = 999.0; hs.y = 999.0;
                hs.vx = 0.0;   hs.vy = 0.0;
                hs.distance_to_robot = 999.0;
                hs.pred_x.assign(N, 999.0);
                hs.pred_y.assign(N, 999.0);
            }
            arr.humans.push_back(hs);
        }

        pub_humans_->publish(arr);
    }

    void publish_markers(const rclcpp::Time& stamp) {
        if (pub_markers_->get_subscription_count() == 0) return;

        visualization_msgs::msg::MarkerArray ma;
        const auto& tracks = tracker_->confirmed_tracks();

        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(del);

        int id = 0;
        for (const auto& t : tracks) {
            const auto pred = t.predict_horizon(DT, N);
            const auto lifetime = rclcpp::Duration::from_seconds(0.5);

            visualization_msgs::msg::Marker body;
            body.header.frame_id = odom_frame_;
            body.header.stamp    = stamp;
            body.ns              = "humans";
            body.id              = id++;
            body.type            = visualization_msgs::msg::Marker::CYLINDER;
            body.action          = visualization_msgs::msg::Marker::ADD;
            body.pose.position.x = t.state[0];
            body.pose.position.y = t.state[1];
            body.pose.position.z = 0.9;
            body.pose.orientation.w = 1.0;
            body.scale.x = 0.5; body.scale.y = 0.5; body.scale.z = 1.8;
            body.color.r = 1.0f; body.color.g = 0.3f; body.color.b = 0.0f; body.color.a = 0.8f;
            body.lifetime = lifetime;
            ma.markers.push_back(body);

            visualization_msgs::msg::Marker line;
            line.header  = body.header;
            line.ns      = "pred_lines";
            line.id      = id++;
            line.type    = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action  = visualization_msgs::msg::Marker::ADD;
            line.scale.x = 0.04f;
            line.color.r = 1.0f; line.color.g = 0.8f; line.color.b = 0.0f; line.color.a = 0.6f;
            line.lifetime = lifetime;
            {
                geometry_msgs::msg::Point p0;
                p0.x = t.state[0]; p0.y = t.state[1]; p0.z = 0.1;
                line.points.push_back(p0);
                for (int k = 0; k < N; ++k) {
                    geometry_msgs::msg::Point pk;
                    pk.x = pred.x[k]; pk.y = pred.y[k]; pk.z = 0.1;
                    line.points.push_back(pk);
                }
            }
            ma.markers.push_back(line);

            for (int k = 5; k < N; k += 6) {
                visualization_msgs::msg::Marker sphere;
                sphere.header  = body.header;
                sphere.ns      = "pred_uncertainty";
                sphere.id      = id++;
                sphere.type    = visualization_msgs::msg::Marker::SPHERE;
                sphere.action  = visualization_msgs::msg::Marker::ADD;
                sphere.pose.position.x = pred.x[k];
                sphere.pose.position.y = pred.y[k];
                sphere.pose.position.z = 0.1;
                sphere.pose.orientation.w = 1.0;
                const float r = 0.15f + 0.025f * static_cast<float>(k);
                sphere.scale.x = r; sphere.scale.y = r; sphere.scale.z = 0.05f;
                const float alpha = 0.45f - 0.35f * static_cast<float>(k) / N;
                sphere.color.r = 1.0f; sphere.color.g = 0.6f; sphere.color.b = 0.0f;
                sphere.color.a = std::max(0.05f, alpha);
                sphere.lifetime = lifetime;
                ma.markers.push_back(sphere);
            }
        }

        pub_markers_->publish(ma);
    }

    PreprocessorParams preproc_params_;
    ClustererParams    cluster_params_;

    std::unique_ptr<KalmanTracker> tracker_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string odom_frame_;

    float robot_x_{0.f};
    float robot_y_{0.f};
    float obstacle_radius_max_{4.f};
    std::vector<DetectedHuman> latest_obstacles_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       sub_odom_;
    rclcpp_lifecycle::LifecyclePublisher<
        social_navigation_msgs::msg::HumanArray>::SharedPtr         pub_humans_;
    rclcpp_lifecycle::LifecyclePublisher<
        visualization_msgs::msg::MarkerArray>::SharedPtr            pub_markers_;
    rclcpp_lifecycle::LifecyclePublisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr                   pub_clusters_;
};

} // namespace human_tracker

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<human_tracker::HumanTrackerNode>(rclcpp::NodeOptions{});
    exec.add_node(node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
