"""Launch-Datei zum Starten aller Scannerkomponenten."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("piscanner_bringup"))
    parameter_file = package_share / "config" / "scanner_parameters.yaml"

    save_directory_arg = DeclareLaunchArgument(
        "save_directory",
        default_value="~/piscanner_data",
        description="Verzeichnis zum Ablegen der Scans",
    )

    motor_node = Node(
        package="piscanner_control",
        executable="stepper_motor_node",
        name="stepper_motor_node",
        output="screen",
        parameters=[str(parameter_file)],
    )

    perception_params = {
        "save_directory": LaunchConfiguration("save_directory"),
    }

    pointcloud_node = Node(
        package="piscanner_perception",
        executable="pointcloud_builder_node",
        name="pointcloud_builder_node",
        output="screen",
        parameters=[str(parameter_file), perception_params],
        remappings=[
            ("/piscanner_camera/panorama", "/camera/image_raw"),
        ],
    )

    return LaunchDescription(
        [
            save_directory_arg,
            motor_node,
            pointcloud_node,
        ]
    )
