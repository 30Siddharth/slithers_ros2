"""
slithers.launch.py
==================
ROS 2 (Humble) launch file for the SLITHERS IK planner node.

Usage
-----
Default launch (holonomic base, 3 paths):
    ros2 launch slithers_ros2 slithers.launch.py

Override robot type:
    ros2 launch slithers_ros2 slithers.launch.py robot_type:=husky

Override number of paths:
    ros2 launch slithers_ros2 slithers.launch.py num_paths:=4

Override multiple arguments:
    ros2 launch slithers_ros2 slithers.launch.py robot_type:=husky num_paths:=2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Declare overridable launch arguments
    # ------------------------------------------------------------------
    robot_type_arg = DeclareLaunchArgument(
        name="robot_type",
        default_value="holo",
        description='Robot type: "holo" (holonomic Husky+UR5e) or "husky" '
                    "(non-holonomic Husky+UR5e)",
        choices=["holo", "husky"],
    )

    num_paths_arg = DeclareLaunchArgument(
        name="num_paths",
        default_value="3",
        description="Number of reference-path trials to run (1-indexed)",
    )

    # ------------------------------------------------------------------
    # slithers_node
    # ------------------------------------------------------------------
    slithers_node = Node(
        package="slithers_ros2",
        executable="slithers_node",
        name="slithers_node",
        output="screen",
        # emulate_tty keeps RCLCPP_INFO colours in the terminal
        emulate_tty=True,
        parameters=[
            {
                # Robot selection — overridable via launch argument
                "robot_type": LaunchConfiguration("robot_type"),

                # Time-stepping parameters (fixed defaults from main.m)
                "dt":   0.1,
                "time": 20.0,

                # Number of path trials — overridable via launch argument
                "num_paths": LaunchConfiguration("num_paths"),

                # IK weighting: error, joint-velocity regularisation
                "lambda_e": [25.0, 25.0, 25.0, 25.0, 25.0, 25.0],
                "lambda_j": 0.001,

                # Per-joint velocity weights (husky: 8-DOF, holo: 9-DOF)
                "lambda_v_husky": [1.0, 1.0, 0.25, 0.25,
                                   0.1, 0.1, 0.1,  0.1],
                "lambda_v_holo":  [1.0, 1.0, 1.0,  0.25, 0.25,
                                   0.1, 0.1, 0.1,  0.1],
            }
        ],
    )

    return LaunchDescription([
        robot_type_arg,
        num_paths_arg,
        slithers_node,
    ])
