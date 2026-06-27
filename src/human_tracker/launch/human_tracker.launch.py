from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share    = get_package_share_directory("human_tracker")
    cfg      = os.path.join(share, "config", "human_tracker.yaml")
    rviz_cfg = os.path.join(share, "config", "human_tracker.rviz")

    tracker_node = Node(
        package="human_tracker",
        executable="human_tracker_node",
        name="human_tracker",
        parameters=[cfg, {"use_sim_time": True}],
        output="screen",
    )

    mpc_node = Node(
        package="social_mpc",
        executable="social_mpc_node",
        name="social_mpc",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    configure_cmd = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/human_tracker", "configure"],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/social_mpc", "configure"],
                output="screen",
            ),
        ],
    )

    activate_cmd = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/human_tracker", "activate"],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/social_mpc", "activate"],
                output="screen",
            ),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
    )

    return LaunchDescription([
        tracker_node,
        mpc_node,
        configure_cmd,
        activate_cmd,
        rviz_node,
    ])
