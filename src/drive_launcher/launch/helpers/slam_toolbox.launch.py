from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    starts the `slam_toolbox` mapping node (to get the `/map` topic)
    """
    depth_image_to_laser_scan_node: Node = Node(
        # note: idk why this is different from how it's named in the src:
        # https://github.com/SteveMacenski/slam_toolbox/blob/ros2/src/slam_toolbox_async_node.cpp
        #
        # oh well...
        executable="async_slam_toolbox_node",
        package="slam_toolbox",
        parameters=[
            {
                "odom_frame": "odom",
                "map_frame": "map",
                "base_frame": "base_link",
                "scan_topic": "/scan",
            }
        ],
        remappings=[
            ("scan", "/scan"),
        ],
        output="screen",
    )

    return LaunchDescription([depth_image_to_laser_scan_node])
