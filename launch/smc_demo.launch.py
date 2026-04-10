"""Launch file for smc_demo: starts SMC controller node and RViz2."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("smc_demo")
    rviz_config = os.path.join(pkg_share, "config", "smc_demo.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("initial_x", default_value="8.0"),
            DeclareLaunchArgument("initial_y", default_value="6.0"),
            DeclareLaunchArgument("target_x", default_value="0.0"),
            DeclareLaunchArgument("target_y", default_value="0.0"),
            DeclareLaunchArgument("use_saturation", default_value="true"),
            DeclareLaunchArgument("k_gain", default_value="8.0"),
            DeclareLaunchArgument("lambda_gain", default_value="4.0"),
            DeclareLaunchArgument("phi", default_value="0.2"),
            DeclareLaunchArgument("enable_comparison", default_value="true"),
            DeclareLaunchArgument("disturbance_enabled", default_value="true"),
            DeclareLaunchArgument("disturbance_start", default_value="1.5"),
            DeclareLaunchArgument("disturbance_duration", default_value="0.35"),
            DeclareLaunchArgument("disturbance_fx", default_value="3.0"),
            DeclareLaunchArgument("disturbance_fy", default_value="-4.0"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            Node(
                package="smc_demo",
                executable="smc_controller",
                name="smc_controller",
                output="screen",
                parameters=[
                    {
                        "initial_x": LaunchConfiguration("initial_x"),
                        "initial_y": LaunchConfiguration("initial_y"),
                        "target_x": LaunchConfiguration("target_x"),
                        "target_y": LaunchConfiguration("target_y"),
                        "use_saturation": LaunchConfiguration("use_saturation"),
                        "k_gain": LaunchConfiguration("k_gain"),
                        "lambda_gain": LaunchConfiguration("lambda_gain"),
                        "phi": LaunchConfiguration("phi"),
                        "enable_comparison": LaunchConfiguration("enable_comparison"),
                        "disturbance_enabled": LaunchConfiguration("disturbance_enabled"),
                        "disturbance_start": LaunchConfiguration("disturbance_start"),
                        "disturbance_duration": LaunchConfiguration("disturbance_duration"),
                        "disturbance_fx": LaunchConfiguration("disturbance_fx"),
                        "disturbance_fy": LaunchConfiguration("disturbance_fy"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("start_rviz")),
                output="screen",
            ),
        ]
    )
