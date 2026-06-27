#include "social_mpc/mpc_solver.hpp"
#include "social_mpc/robot_kinematics.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <social_navigation_msgs/msg/human_array.hpp>

#include <cmath>
#include <limits>
#include <optional>

namespace social_mpc {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

constexpr double GOAL_REACHED_DIST = 0.3;   // [m]
constexpr double DT                = 0.1;   // must match generate_social_mpc.py
constexpr double HUMAN_STOP_DIST        = 0.8;  // [m] full stop
constexpr double HUMAN_SLOW_DIST        = 1.4;  // [m] proximity slow-down
constexpr double CROSSING_YIELD_DIST    = 2.5;  // [m] closest-approach distance that triggers yield
constexpr double CROSSING_YIELD_HORIZON = 3.0;  // [s] lookahead for crossing detection

class SocialMpcNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit SocialMpcNode(const rclcpp::NodeOptions& opts)
        : rclcpp_lifecycle::LifecycleNode("social_mpc", opts) {
        declare_parameter<std::string>("odom_topic",    "/chassis/odom");
        declare_parameter<std::string>("humans_topic",  "/humans/tracked");
        declare_parameter<std::string>("goal_topic",    "/goal_pose");
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
        try {
            solver_ = std::make_unique<MpcSolver>();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "MPC solver init failed: %s", e.what());
            return CallbackReturn::FAILURE;
        }

        pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>(
            get_parameter("cmd_vel_topic").as_string(), rclcpp::QoS(1));
        pub_path_ = create_publisher<nav_msgs::msg::Path>(
            "/social_mpc/planned_path", rclcpp::QoS(1));
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
        pub_cmd_vel_->on_activate();
        pub_path_->on_activate();

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            get_parameter("odom_topic").as_string(), rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_cb(msg); });

        sub_humans_ = create_subscription<social_navigation_msgs::msg::HumanArray>(
            get_parameter("humans_topic").as_string(), rclcpp::QoS(5),
            [this](social_navigation_msgs::msg::HumanArray::SharedPtr msg) {
                latest_humans_ = msg;
            });

        sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            get_parameter("goal_topic").as_string(), rclcpp::QoS(1),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { goal_cb(msg); });

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(DT * 1000)),
            [this]() { timer_cb(); });

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
        timer_.reset();
        sub_odom_.reset();
        sub_humans_.reset();
        sub_goal_.reset();
        publish_stop();
        pub_cmd_vel_->on_deactivate();
        pub_path_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
        solver_.reset();
        pub_cmd_vel_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
        return CallbackReturn::SUCCESS;
    }

private:
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr& msg) {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;

        // Extract yaw from quaternion
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        robot_state_[0] = p.x;
        robot_state_[1] = p.y;
        robot_state_[2] = std::atan2(siny_cosp, cosy_cosp);
        have_odom_ = true;
    }

    void goal_cb(const geometry_msgs::msg::PoseStamped::SharedPtr& msg) {
        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;
        const double siny = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        goal_ = Eigen::Vector3d{p.x, p.y, std::atan2(siny, cosy)};
        RCLCPP_INFO(get_logger(), "New goal: (%.2f, %.2f)", p.x, p.y);
    }

    void timer_cb() {
        if (!have_odom_ || !goal_) {
            publish_stop();
            return;
        }

        // Check goal reached
        const double dist = std::hypot(robot_state_[0] - (*goal_)[0],
                                       robot_state_[1] - (*goal_)[1]);
        if (dist < GOAL_REACHED_DIST) {
            publish_stop();
            goal_.reset();
            RCLCPP_INFO(get_logger(), "Goal reached.");
            return;
        }

        // Build human params: per stage k → [xh0,yh0,...,xh4,yh4]
        MpcInput input;
        input.x0   = robot_state_;
        input.goal = *goal_;
        input.human_params.fill(999.0); // default: dummy humans far away

        int real_humans = 0;
        if (latest_humans_) {
            const auto& humans = latest_humans_->humans;
            real_humans = static_cast<int>(latest_humans_->num_confirmed);
            for (int k = 0; k < MPC_N; ++k) {
                for (int i = 0; i < MPC_M; ++i) {
                    // Positions: first M*2 slots
                    const int pos_base = k * MPC_NP + i * 2;
                    // Velocities: next M*2 slots (offset by M*2 within stage)
                    const int vel_base = k * MPC_NP + MPC_M * 2 + i * 2;
                    if (i < static_cast<int>(humans.size()) &&
                        k < static_cast<int>(humans[i].pred_x.size())) {
                        input.human_params[pos_base]     = humans[i].pred_x[k];
                        input.human_params[pos_base + 1] = humans[i].pred_y[k];
                        input.human_params[vel_base]     = humans[i].vx;
                        input.human_params[vel_base + 1] = humans[i].vy;
                    } else {
                        input.human_params[vel_base]     = 0.0;
                        input.human_params[vel_base + 1] = 0.0;
                    }
                }
            }
        }

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "MPC: humans=%d dist_to_goal=%.2fm", real_humans, dist);

        const MpcOutput out = solver_->solve(input);

        if (out.status != 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "MPC solver status: %d (humans tracked: %d)", out.status, real_humans);
        }

        // Publish planned trajectory
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp    = now();
        path_msg.header.frame_id = "odom";
        for (const auto& state : out.trajectory) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = state[0];
            ps.pose.position.y = state[1];
            path_msg.poses.push_back(ps);
        }
        pub_path_->publish(path_msg);

        // Reactive safety: proximity stop/slow + crossing yield
        double min_human_dist = std::numeric_limits<double>::max();
        double yield_scale    = 1.0;

        if (latest_humans_) {
            const uint32_t n = latest_humans_->num_confirmed;
            for (uint32_t i = 0; i < n && i < latest_humans_->humans.size(); ++i) {
                const auto& h = latest_humans_->humans[i];
                min_human_dist = std::min(min_human_dist,
                    static_cast<double>(h.distance_to_robot));

                // Crossing yield: slow robot when a human will cross its straight-ahead path.
                // Uses time-to-closest-approach (TCA) between robot and human.
                const double rv  = out.u0[0];
                const double rvx = rv * std::cos(robot_state_[2]);
                const double rvy = rv * std::sin(robot_state_[2]);
                const double dx  = robot_state_[0] - h.x;
                const double dy  = robot_state_[1] - h.y;
                const double dvx = rvx - h.vx;
                const double dvy = rvy - h.vy;
                const double dv2 = dvx * dvx + dvy * dvy;

                if (dv2 > 0.01) {
                    const double t_ca = -(dx * dvx + dy * dvy) / dv2;
                    if (t_ca > 0.0 && t_ca < CROSSING_YIELD_HORIZON) {
                        const double d_ca = std::hypot(dx + t_ca * dvx, dy + t_ca * dvy);
                        if (d_ca < CROSSING_YIELD_DIST) {
                            // Yield: slow proportionally — closer approach = more slowdown
                            const double urgency = 1.0 - d_ca / CROSSING_YIELD_DIST;
                            const double time_factor = 1.0 - t_ca / CROSSING_YIELD_HORIZON;
                            yield_scale = std::min(yield_scale,
                                1.0 - 0.95 * urgency * time_factor);
                        }
                    }
                }
            }
        }

        if (min_human_dist <= HUMAN_STOP_DIST) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "Safety stop: human at %.2fm", min_human_dist);
            publish_stop();
            return;
        }

        double vel_scale = yield_scale;
        if (min_human_dist < HUMAN_SLOW_DIST) {
            const double prox_scale = (min_human_dist - HUMAN_STOP_DIST) /
                                      (HUMAN_SLOW_DIST - HUMAN_STOP_DIST);
            vel_scale = std::min(vel_scale, prox_scale);
        }

        const RobotControl u_clamped = clamp_control(out.u0, RobotParams{});
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = u_clamped[0] * vel_scale;
        cmd.angular.z = u_clamped[1];  // full angular velocity for steering away
        pub_cmd_vel_->publish(cmd);
    }

    void publish_stop() {
        geometry_msgs::msg::Twist cmd;
        pub_cmd_vel_->publish(cmd);
    }

    std::unique_ptr<MpcSolver>                                           solver_;
    rclcpp::TimerBase::SharedPtr                                         timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             sub_odom_;
    rclcpp::Subscription<social_navigation_msgs::msg::HumanArray>::SharedPtr sub_humans_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr     sub_goal_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr  pub_cmd_vel_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr        pub_path_;

    Eigen::Vector3d                                    robot_state_{Eigen::Vector3d::Zero()};
    std::optional<Eigen::Vector3d>                     goal_;
    social_navigation_msgs::msg::HumanArray::SharedPtr latest_humans_;
    bool                                               have_odom_{false};
};

} // namespace social_mpc

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<social_mpc::SocialMpcNode>(rclcpp::NodeOptions{});
    exec.add_node(node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
