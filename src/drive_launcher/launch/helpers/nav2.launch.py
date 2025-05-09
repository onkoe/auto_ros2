from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import (
    ComposableNodeContainer,
    LoadComposableNodes,
)
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

CONTAINER_NAME: str = "nav2_container"
"""The name of the Nav2 container we're starting."""


def generate_launch_description() -> LaunchDescription:
    # whether to use the simulator's time instead of ROS 2
    use_sim_time: LaunchConfiguration = LaunchConfiguration("use_sim_time")

    # remap tf2 stuff to work within the container
    remappings: list[tuple[str, str]] = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    # grab our (yaml) configuration file for Nav2
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")
    params_file: PathJoinSubstitution = PathJoinSubstitution(
        [
            pkg_drive_launcher,
            "params",
            "nav2.yaml",
        ]
    )

    # screw with our yaml to make it work with composition.
    #
    # note: there's a damn good reason this comes with Nav2! i have an entire
    # library of GitHub Issues tickets complaining about the setup. lmao
    #
    # - composition has trouble with params: https://github.com/ros-navigation/navigation2/issues/4011
    # - nav2 regression on Humble + Jazzy: https://github.com/turtlebot/turtlebot4/discussions/550
    # - nav2 launches the `local_costmap` and `global_costmap` on their own,
    #       but attempting to do so yourself SUCKS: https://github.com/ros-navigation/navigation2/issues/2828
    # - as i said above... https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp#L65
    # - to conf costmaps in container, need to pass params to the container
    #       itself: https://github.com/ros-navigation/navigation2/issues/4011
    configured_yaml: RewrittenYaml = RewrittenYaml(
        source_file=params_file,
        root_key=None,  # required to avoid overlapping namespace
        param_rewrites={"use_sim_time": use_sim_time},
        convert_types=True,  # parse yaml types into literals
    )

    # we need to make a "container" for the nodes to exist in. this container
    # MUST be defined inside this file - otherwise, we can't remap the params
    # correctly :(
    #
    # using a container allows nodes to share memory - node composition on Nav2
    # usually speeds things up.
    #
    # see: https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html#component-container-types
    nav2_container: ComposableNodeContainer = ComposableNodeContainer(
        name=CONTAINER_NAME,
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # note: `_mt` means we use one executor
        parameters=[configured_yaml],  # IMPORTANT! params FAIL w/o it...
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # grab our list of lifecycle nodes
    lifecycle_nodes: list[ComposableNode] = [
        ComposableNode(
            package="nav2_controller",
            plugin="nav2_controller::ControllerServer",
            name="controller_server",
            parameters=[configured_yaml],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        ComposableNode(
            package="nav2_smoother",
            plugin="nav2_smoother::SmootherServer",
            name="smoother_server",
            parameters=[configured_yaml],
            remappings=remappings,
        ),
        ComposableNode(
            package="nav2_planner",
            plugin="nav2_planner::PlannerServer",
            name="planner_server",
            parameters=[configured_yaml],
            remappings=remappings,
        ),
        ComposableNode(
            package="nav2_behaviors",
            plugin="behavior_server::BehaviorServer",
            name="behavior_server",
            parameters=[configured_yaml],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        ComposableNode(
            package="nav2_bt_navigator",
            plugin="nav2_bt_navigator::BtNavigator",
            name="bt_navigator",
            parameters=[configured_yaml],
            remappings=remappings,
        ),
        ComposableNode(
            package="nav2_waypoint_follower",
            plugin="nav2_waypoint_follower::WaypointFollower",
            name="waypoint_follower",
            parameters=[configured_yaml],
            remappings=remappings,
        ),
        ComposableNode(
            package="nav2_velocity_smoother",
            plugin="nav2_velocity_smoother::VelocitySmoother",
            name="velocity_smoother",
            parameters=[configured_yaml],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        ComposableNode(
            package="nav2_collision_monitor",
            plugin="nav2_collision_monitor::CollisionMonitor",
            name="collision_monitor",
            parameters=[configured_yaml],
            remappings=remappings,
        ),
    ]

    # make the lifecycle manager to support the nodes above after loading.
    #
    # we'll spawn it first.
    lifecycle_manager: ComposableNode = ComposableNode(
        package="nav2_lifecycle_manager",
        plugin="nav2_lifecycle_manager::LifecycleManager",
        name="lifecycle_manager_navigation",
        parameters=[  # two parameter sets:
            {
                "autostart": True,
                "node_names": [n.node_name for n in lifecycle_nodes],
            },
            configured_yaml,
        ],  # plus all common params
        remappings=remappings,
    )

    # now, we'll load nodes in the container only after the server can recv.
    # them. (this action blocks until the container is ready!)
    #
    # without this, we can't pass our config (via parameters) correctly.
    load_composable_nodes: LoadComposableNodes = LoadComposableNodes(
        target_container=CONTAINER_NAME,
        composable_node_descriptions=lifecycle_nodes + [lifecycle_manager],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            nav2_container,
            load_composable_nodes,
        ]
    )
