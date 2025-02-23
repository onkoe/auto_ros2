import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher

import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    package_dir = get_package_share_directory("soro_simulator_driver")
    robot_description_path = os.path.join(package_dir, "resource", "remi.urdf")

    webots = WebotsLauncher(
        world=os.path.join(package_dir, "worlds", "base_world.wbt")
    )

    my_robot_driver = WebotsController(
        robot_name="REMI",
        parameters=[
            {"robot_description": robot_description_path},
        ],
    )

    # stop everything when webots goes down
    exit_handler: RegisterEventHandler = launch.actions.RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    # make an instance of the navigator node!
    navigator: Node = Node(
        executable="navigator_node",
        package="navigator_node",
        name="navigator",
        # parameters=[
        #     {
        #         "navigator_parameters": LaunchConfiguration(
        #             "navigator_parameters"
        #         )
        #     }
        # ],
    )

    return LaunchDescription(
        [webots, my_robot_driver, exit_handler, navigator],
    )
