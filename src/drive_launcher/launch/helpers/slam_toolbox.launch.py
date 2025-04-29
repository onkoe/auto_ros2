from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    starts the `slam_toolbox` mapping node (to get the `/map` topic)
    """
    pkg_slam_toolbox: FindPackageShare = FindPackageShare("slam_toolbox")

    use_sim_time: LaunchConfiguration = LaunchConfiguration(
        "use_sim_time", default="false"
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_slam_toolbox,
                        "launch",
                        "online_async_launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("use_sim_time", use_sim_time),
            ("slam_params_file", _get_conf()),
            ("use_lifecycle_manager", "true"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
            ),
            slam_toolbox,
        ]
    )


def _get_conf() -> PathJoinSubstitution:
    return PathJoinSubstitution(
        [
            FindPackageShare("drive_launcher"),
            "params",
            "slam_toolbox.yaml",
        ]
    )
