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
        declare_parameter<double>("height_min",        0.3);
        declare_parameter<double>("height_max",        2.0);
        declare_parameter<double>("radius_max",       10.0);
        declare_parameter<double>("voxel_leaf",        0.05);
        declare_parameter<double>("cluster_tolerance", 0.3);
        declare_parameter<int>   ("min_cluster_size",  10);
        declare_parameter<int>   ("max_cluster_size",  500);
        declare_parameter<double>("gate_distance",     1.0);
        declare_parameter<int>   ("confirm_frames",    3);
        declare_parameter<int>   ("lost_frames",       10);
        declare_parameter<std::string>("lidar_topic",  "/front_3d_lidar/lidar_points");
        declare_parameter<std::string>("odom_topic",   "/chassis/odom");
        declare_parameter<std::string>("odom_frame",   "odom");
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
        PreprocessorParams pp;
        pp.height_min = static_cast<float>(get_parameter("height_min").as_double());
        pp.height_max = static_cast<float>(get_parameter("height_max").as_double());
        pp.radius_max = static_cast<float>(get_parameter("radius_max").as_double());
        pp.voxel_leaf = static_cast<float>(get_parameter("voxel_leaf").as_double());
        preproc_params_ = pp;

        ClustererParams cp;
        cp.cluster_tolerance = static_cast<float>(get_parameter("cluster_tolerance").as_double());
        cp.min_cluster_size  = get_parameter("min_cluster_size").as_int();
        cp.max_cluster_size  = get_parameter("max_cluster_size").as_int();
        cluster_params_ = cp;

        KalmanParams kp;
        kp.dt            = DT;
        kp.gate_dist     = get_parameter("gate_distance").as_double();
        kp.confirm_frames = get_parameter("confirm_frames").as_int();
        kp.lost_frames   = get_parameter("lost_frames").as_int();
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

        const auto detections = extract_humans(filtered, cluster_params_);
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

        for (int i = 0; i < M_MAX; ++i) {
            social_navigation_msgs::msg::HumanState hs;
            hs.header = arr.header;

            if (i < static_cast<int>(sorted.size())) {
                const Track& t = *sorted[i];
                const auto pred = t.predict_horizon(DT, N);
                hs.id               = t.id;
                hs.x                = t.state[0];
                hs.y                = t.state[1];
                hs.vx               = t.state[2];
                hs.vy               = t.state[3];
                hs.distance_to_robot = t.distance_to(robot_x_, robot_y_);
                hs.pred_x.assign(pred.x.begin(), pred.x.end());
                hs.pred_y.assign(pred.y.begin(), pred.y.end());
            } else {
                // Dummy — place far away so MPC constraint is inactive
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

        // Delete old markers first
        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(del);

        int id = 0;
        for (const auto& t : tracks) {
            // Cylinder marker for human body
            visualization_msgs::msg::Marker m;
            m.header.frame_id = odom_frame_;
            m.header.stamp    = stamp;
            m.ns              = "humans";
            m.id              = id++;
            m.type            = visualization_msgs::msg::Marker::CYLINDER;
            m.action          = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = t.state[0];
            m.pose.position.y = t.state[1];
            m.pose.position.z = 0.9;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 1.8;
            m.color.r = 1.0f; m.color.g = 0.3f; m.color.b = 0.0f; m.color.a = 0.7f;
            m.lifetime = rclcpp::Duration::from_seconds(0.5);
            ma.markers.push_back(m);

            // Velocity arrow
            visualization_msgs::msg::Marker arrow;
            arrow.header = m.header;
            arrow.ns     = "velocities";
            arrow.id     = id++;
            arrow.type   = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            geometry_msgs::msg::Point start, end;
            start.x = t.state[0]; start.y = t.state[1]; start.z = 1.0;
            end.x   = t.state[0] + t.state[2];
            end.y   = t.state[1] + t.state[3];
            end.z   = 1.0;
            arrow.points = {start, end};
            arrow.scale.x = 0.05; arrow.scale.y = 0.1;
            arrow.color.r = 1.0f; arrow.color.g = 1.0f; arrow.color.b = 0.0f; arrow.color.a = 0.9f;
            arrow.lifetime = rclcpp::Duration::from_seconds(0.5);
            ma.markers.push_back(arrow);
        }

        pub_markers_->publish(ma);
    }

    // Preprocessor / clusterer params
    PreprocessorParams preproc_params_;
    ClustererParams    cluster_params_;

    // Tracker
    std::unique_ptr<KalmanTracker> tracker_;

    // TF
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string odom_frame_;

    // Robot pose (updated by odom subscriber)
    float robot_x_{0.f};
    float robot_y_{0.f};

    // Publishers / subscribers
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
