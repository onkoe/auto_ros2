from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    starts the `robot_state_publisher::robot_state_publisher` node.
    """
    # grab the rover's udrf
    robot_desc: dict[str, ParameterValue] = _grab_robot_description()

    # the `robot_state_publisher` node basically takes the various transform
    # differences defined in a URDF file and tells other nodes where stuff on
    # your robot is.
    #
    # so, on the Rover, we use this for, e.g., offsetting the GPS by the
    # correct distance for consistent mapping
    rover_state_publisher_node: Node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_desc],
    )

    return LaunchDescription([rover_state_publisher_node])


def _grab_robot_description() -> dict[str, ParameterValue]:
    """grabs the robot desc."""

    # make the description
    robot_description_content: Command = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("simulator"),
                    "resource",
                    "rover.urdf.xacro.xml",
                ]
            ),
        ]
    )

    # return it in a dict
    return {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
