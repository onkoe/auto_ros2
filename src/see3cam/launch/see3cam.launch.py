from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import (
    Node,
)
from launch_ros.actions.load_composable_nodes import (
    ComposableNode,
    ComposableNodeContainer,
)
from launch_ros.substitutions.find_package import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    see3cam_pkg_path: str = get_package_share_directory("see3cam")

    cap: Node = Node(
        name="v4l2_mono_image",
        package="v4l2_camera",
        executable="v4l2_camera_node",
        remappings=[
            ("image_raw", "/sensors/mono_image/raw"),
            ("camera_info", "/sensors/mono_image/camera_info"),
        ],
        parameters=[
            PathJoinSubstitution([see3cam_pkg_path, "params", "v4l2_camera.yaml"]),
            {"camera_info_url": f"file://{see3cam_pkg_path}/params/camera_info.yaml"},
        ],
    )

    rectifier: ComposableNode = ComposableNode(
        name="rectify_mono_image",
        package="image_proc",
        plugin="image_proc::RectifyNode",
        remappings=[
            ("image", "/sensors/mono_image/raw"),
            ("camera_info", "/sensors/mono_image/camera_info"),
            ("image_rect", "/sensors/mono_image"),
        ],
    )

    composable_nodes: list[ComposableNode] = [
        rectifier,
    ]

    container = ComposableNodeContainer(
        name="image_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    return LaunchDescription([cap, container])
