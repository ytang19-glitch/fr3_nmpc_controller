from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("fr3_nmpc_controller"))
    parameters = package_share / "config" / "reference_monitor.yaml"

    return LaunchDescription(
        [
            Node(
                package="fr3_nmpc_controller",
                executable="reference_monitor_node",
                name="fr3_nmpc_reference_monitor",
                output="screen",
                parameters=[str(parameters)],
            )
        ]
    )
