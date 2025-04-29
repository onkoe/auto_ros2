from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    # whether to use the simulator's time instead of ROS 2
    use_sim_time = LaunchConfiguration("use_sim_time")

    # TODO(bray): got not clue what these do lol
    autostart = LaunchConfiguration("autostart")
    container_name = LaunchConfiguration("container_name")
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")
    params_file: PathJoinSubstitution = PathJoinSubstitution(
        [
            pkg_drive_launcher,
            "params",
            "nav2.yaml",
        ]
    )

    lifecycle_nodes: list[str] = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
        "waypoint_follower",
    ]

    # we'll start all these nodes using "composition"
    #
    # for more info, see:
    #
    # https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html
    composable_nodes = [
        SetParameter("use_sim_time", use_sim_time),
        LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package="nav2_controller",
                    plugin="nav2_controller::ControllerServer",
                    name="controller_server",
                    parameters=[params_file],
                    remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                ),
                ComposableNode(
                    package="nav2_smoother",
                    plugin="nav2_smoother::SmootherServer",
                    name="smoother_server",
                    parameters=[params_file],
                    remappings=remappings,
                ),
                ComposableNode(
                    package="nav2_planner",
                    plugin="nav2_planner::PlannerServer",
                    name="planner_server",
                    parameters=[params_file],
                    remappings=remappings,
                ),
                ComposableNode(
                    package="nav2_behaviors",
                    plugin="behavior_server::BehaviorServer",
                    name="behavior_server",
                    parameters=[params_file],
                    remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                ),
                ComposableNode(
                    package="nav2_bt_navigator",
                    plugin="nav2_bt_navigator::BtNavigator",
                    name="bt_navigator",
                    parameters=[params_file],
                    remappings=remappings,
                ),
                ComposableNode(
                    package="nav2_waypoint_follower",
                    plugin="nav2_waypoint_follower::WaypointFollower",
                    name="waypoint_follower",
                    parameters=[params_file],
                    remappings=remappings,
                ),
                ComposableNode(
                    package="nav2_velocity_smoother",
                    plugin="nav2_velocity_smoother::VelocitySmoother",
                    name="velocity_smoother",
                    parameters=[params_file],
                    remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                ),
                ComposableNode(
                    package="nav2_collision_monitor",
                    plugin="nav2_collision_monitor::CollisionMonitor",
                    name="collision_monitor",
                    parameters=[params_file],
                    remappings=remappings,
                ),
                ComposableNode(
                    package="nav2_lifecycle_manager",
                    plugin="nav2_lifecycle_manager::LifecycleManager",
                    name="lifecycle_manager_navigation",
                    parameters=[
                        {
                            "autostart": autostart,
                            "node_names": lifecycle_nodes,
                        }
                    ],
                ),
            ],
        ),
    ]

    ld: LaunchDescription = LaunchDescription()
    for n in composable_nodes:
        ld.add_action(n)

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
    )
    return ld
