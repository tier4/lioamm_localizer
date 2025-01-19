from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    global_initializer_config_path = PathJoinSubstitution(
        [FindPackageShare("global_initializer"), "config", "global_initializer.params.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="global_initializer",
                executable="global_initializer_node",
                name="global_initializer",
                output="screen",
                remappings=[("points_raw", "/sensing/lidar/left_upper/pointcloud_raw"), ("points_map", "map")],
                parameters=[global_initializer_config_path],
            ),
        ]
    )
