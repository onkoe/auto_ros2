from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    returns a node that convert our depth camera outputs into fake laserscans
    lol
    """
    depth_image_to_laser_scan_node: Node = Node(
        executable="depthimage_to_laserscan_node",
        package="depthimage_to_laserscan",
        parameters=[
            {
                "scan_time": 0.033,  # seconds
                "range_min": 0.45,  # meters
                "range_max": 10.0,
                "scan_height": 1,
                "output_frame": "camera_depth_frame",
            }
        ],
        remappings=[
            ("depth/camera_info", "/sensors/depth_image/camera_info"),
            ("depth/image_raw", "/sensors/depth_image"),
            ("scan", "/scan"),  # otherwise, it outputs locally
        ],
    )

    return LaunchDescription([depth_image_to_laser_scan_node])
