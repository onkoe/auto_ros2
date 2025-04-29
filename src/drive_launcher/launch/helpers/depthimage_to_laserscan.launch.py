from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.frontend.parse_substitution import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    returns a node that convert our depth camera outputs into fake laserscans
    lol
    """
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")
    use_sim_time = LaunchConfiguration("use_sim_time")

    depth_image_to_laser_scan_node: Node = Node(
        executable="depthimage_to_laserscan_node",
        package="depthimage_to_laserscan",
        arguments=[
            PathJoinSubstitution(
                [
                    pkg_drive_launcher,
                    "params",
                    "depthimage_to_laserscan.yaml",
                ]
            )
        ],
        parameters=[
            {
                "scan_time": 0.033,  # seconds
                "range_min": 0.02,  # meters
                "range_max": 35.0,
                "scan_height": 1,
                "output_frame": "camera_depth_frame",
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[
            ("/depth_camera_info", "/sensors/depth_image/camera_info"),
            ("/depth", "/sensors/depth_image"),
            ("scan", "/scan"),  # otherwise, it outputs locally
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
            ),
            depth_image_to_laser_scan_node,
        ]
    )
