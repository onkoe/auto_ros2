"""
stuff that the navigator node does:
- takes a coordinate and an indicator of which of the following we're doing: GPS, Aruco, or Object Detection
- turn lights RED (for Autonomous mode)
- no matter what, drive to a GPS coordinate (near a goal)
    - subscribe to GPS topic
    - receive messages repeatedly on current location
    - within class, save coordinate + time recv'd to use later
    - that includes distance calculation (how far are we from coordinate)
        - note: consider using WSG83 instead of flat coordinates for correct result
    - create a wheels publisher
    - based on distance, how far do we need to move, and in which direction?
        - PID controller (DO NOT IMPLEMENT MANUALLY @tyler)
    - send wheel speeds
- if Aruco, we need to subscribe to Aruco topic once coordinate is reached
    - this will return a pose of the marker
    - if marker found, we need to calculate distance to the marker
        - based on distance, we need to calculate how/where to move
    - if marker not found, we need to change strategy
- if Object Detection, we'll do something! but let's not care right now
- CALLBACK:
    - oh we received new gps info! now, let's move wheels, turn on lights, or exit
        - if we have reached coordinate and we are doing GPS, flash green lights, stop Rover
    - oh we received new aruco info! now, let's change strategy
- launches other nodes necessary for requested form of navigation
"""

import dataclasses
import math
import sys
from collections.abc import Coroutine
from dataclasses import dataclass
from enum import Enum
from typing import Any

import rclpy
from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import PoseStamped
from geopy.distance import distance
from loguru import logger as llogger
from rclpy.client import Client
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription
from rclpy.task import Future
from rclpy.time import Time
from rclpy.timer import Timer
from simple_pid import PID
from typing_extensions import override

# sudo apt install ros-jazzy-geographic-msgs
# FIXME: make this use the fr module!
#        seems like there's some kinda Python glue missing...
from custom_interfaces.msg import (
    GpsMessage,
    ImuMessage,
    WheelsMessage,
)
from custom_interfaces.srv._lights import (
    Lights_Request as LightsRequest,
)
from custom_interfaces.srv._lights import (
    Lights_Response as LightsResponse,
)

# from custom_interfaces.msg import ArMessage as ArucoMessage
from .coords import coordinate_from_aruco_pose, get_distance_to_marker
from .search import generate_similar_coordinates

## how long we'll keep the data (DDS).
QUEUE_SIZE: int = 10

SENSOR_TIMEOUT_NS: float = 2.0 * 1_000_000_000
"""
The amount of time until we ignore a message from a sensor.

Using this allows us to ignore old messages, only using new ones.

Measured in nanoseconds.
"""

## minimum distance we need to be within from the coordinate
MIN_GPS_DISTANCE: float = 3.0  # meters
MIN_ARUCO_DISTANCE: float = 2.0  # meters


class NavigationMode(Enum):
    GPS = 0
    ARUCO = 1
    OBJECT_DETECTION = 2


class GoToCoordinateReason(Enum):
    """
    The 'reason' we're going to a coordinate.

    As an argument to `go_to_coordinate`, this type allows it to use the correct
    stopping distance.
    """

    GPS = 0
    ARUCO = 1


@dataclass(kw_only=True)
class NavigationParameters:
    coord: GeoPoint
    """
    The coordinate that either is our goal (if `mode` is `GPS`), or is some
    point around an ArUco/object.
    """

    mode: NavigationMode
    """
    Indicates what the Navigator is doing.

    For example, if we're given `NavigationMode::GPS`, we'll navigate to the given GPS
    coordinate and stop when we're there.
    """
    pk: float = 0.0
    """
    Proportional gain for the PID controller.
    Determines the speed at which the rover will correct itself.
    Default: 0.0
    """
    pi: float = 0.0
    """
    Integral gain for the PID controller.
    Determines how much the rover will correct itself.
    Default: 0.0
    """
    pd: float = 0.0
    """
    Derivative gain for the PID controller.
    Determines how quickly the rover will stop correcting itself.
    Default: 0.0
    """


@dataclass(kw_only=True)
class NavigatorNode(Node):
    _wheels_publisher: Publisher
    """Publish wheels speeds to move the Rover."""
    _gps_subscription: Subscription
    _aruco_subscription: Subscription
    _imu_subscription: Subscription
    """Grabs information from the IMU, which includes compass, acceleration, and orientation."""
    ## service client for lights node
    _lights_client: Client

    _navigator_callback_timer: Timer
    """The timer that runs the main `navigator` logic."""

    _lights_request_future: Future
    """A `Future` that we await to change the light color."""

    _param_value: NavigationParameters | None

    _given_aruco_marker_id: int | None = None
    """
    The ID of the ArUco marker we're looking for.

    We don't use this directly - it's only to make error messages better.
    """

    _times_marker_seen: int = 0

    # these let us check if we've reached stuff
    calculating_aruco_coord: bool = False
    """When this is true, we've reached the coordinate near the goal."""
    goal_reached: bool = False
    """When true, we've reached the object itself."""

    # info about where the rover/target is
    _last_known_rover_coord: GeoPointStamped | None = None
    """The coordinate last recv'd from the GPS."""
    _last_known_marker_coord: GeoPointStamped | None = None
    """Coordinate pair where the target was last known to be located."""

    _curr_marker_transform: PoseStamped | None = None

    _last_known_imu_data: ImuMessage | None = None

    _coordinate_path_queue: list[GeoPoint] = dataclasses.field(
        default_factory=list
    )
    """
    A coordinate queue where the first element of the queue holds whatever coordinate we want to go to next.
    This is initially set to only hold the given coordinate parameter. Then, if we are doing an ArUco or Object Detection task,
    further coordinates are added to the queue in order to determine where the Rover searches next. If we find the coordinate for
    an ArUco tag or object, we add that tag to the front of the queue and generate more search coordinates around that area.
    No matter what task we're performing, we know we are going to navigate to the parameter coordinate. If we're doing the GPS task,
    this will be the only element in the queue.

    If not, we can work off of the current element (which could be the original coordinate, or the estimated coordinate for a tag)
    in order to give ourselves a planned path for each location we want the Rover to go to.

    Then, in the Navigator logic, it follows a simple structure of driving to whatever is at the front, while at the same time,
    looking for the desired ArUco tag/object.
    """

    # TODO: use... not bools for that
    _search_algo_cor: Coroutine[Any, Any, None] | None = None
    """search algo if we're doin it"""
    _go_to_coordinate_cor: Coroutine[Any, Any, None] | None = None
    """
    we replace this with a Coroutine (running async function) when we want
    to go somewhere.

    we can set it to None to cancel it, which allows us to navigate to another
    coordinate, or keep it going and check if it's returned yet to see if it's
    arrived.

    in short, this handles the actual navigation without requiring dense
    logic within the `navigator` function
    """

    _last_searched_coord: GeoPoint | None = None
    """
    The coordinate we were searching at during the previous callback iteration.
    """

    # current gps attribute

    def __init__(self):
        """
        Creates a new `NavigatorNode`.
        """
        super().__init__("navigator_node")
        _ = self.declare_parameter("navigator_parameters", None)

        # try to grab the instructions we're given over parameters.
        #
        # if none are given, this is `None`, and will cause the program to go
        # down
        self._param_value = self.get_parameter("navigator_parameters").value

        if self._param_value is None:
            _ = self.get_logger().error("navigator_parameters is not set!")
            return

        # create a service client for lights node
        self._lights_client = self.create_client(
            srv_type=LightsRequest,
            srv_name="lights",
            qos_profile=QoSProfile(queue_size=QUEUE_SIZE),
        )

        # give it one second to connect
        llogger.debug("waiting for lights service...")
        while not self._lights_client.wait_for_service(timeout_sec=1.0):
            _ = self.get_logger().info(
                "service not available, waiting again..."
            )
        llogger.debug("lights ")

        # turn lights RED immediately upon startup.
        #
        # it's required for competition :)
        lights_info: LightsRequest = LightsRequest()
        lights_info.red = 255
        lights_info.green = 0
        lights_info.blue = 0
        lights_info.flashing = False

        _ = self.send_lights_request(lights_info)

        # create wheels publisher
        self._wheels_publisher = self.create_publisher(
            msg_type=WheelsMessage,
            topic="/controls/wheels",
            qos_profile=QUEUE_SIZE,
        )

        # if in aruco mode, create a subscriber for aruco tracking
        if self._param_value.mode == NavigationMode.ARUCO:
            self._aruco_subscription = self.create_subscription(
                msg_type=PoseStamped,  # TODO: change to wherever the ar message type is
                topic="/aruco",  # TODO: change to whatever the markers topic is
                callback=self.aruco_callback,
                qos_profile=QUEUE_SIZE,
            )

        # connect to our sensors using subscriptions
        self._gps_subscription = self.create_subscription(
            msg_type=GpsMessage,
            topic="/sensors/gps",
            callback=self.gps_callback,
            qos_profile=QUEUE_SIZE,
        )
        self._imu_subscription = self.create_subscription(
            msg_type=ImuMessage,
            topic="/sensors/imu",
            callback=self.imu_callback,
            qos_profile=QUEUE_SIZE,
        )

        # Add the given GPS coordinate to the coordinate queue
        self._coordinate_path_queue.append(self._param_value.coord)
        # Calculate and append search coordinates for GPS coord
        self._coordinate_path_queue.extend(
            generate_similar_coordinates(self._param_value.coord, 10, 5)
        )  # takes source coordinate, radius in meters, and a number of points to generate

        # Run navigator callback every 0.5 seconds
        self._navigator_callback_timer = self.create_timer(0.5, self.navigator)

    # Function to send lights request given a LightsRequest class instance
    def send_lights_request(
        self, lights_info: LightsRequest
    ) -> LightsResponse | None:
        """
        Send a request to the lights service to turn the lights red.
        """
        self._lights_request_future = self._lights_client.call_async(
            lights_info
        )
        rclpy.spin_until_future_complete(self, self._lights_request_future)
        return self._lights_request_future.result()

    def navigator(self):
        """
        TODO: docs...

        Driven by a `rclpy::Timer`.
        """
        # Ensure navigation parameters were specified
        if self._param_value is None:
            llogger.error("Called, but no parameters given.")
            rclpy.shutdown()

        # If goal is reached, turn the node off
        if self.goal_reached:
            # Log message that goal was reached
            _ = self.get_logger().info("Goal reached!")
            # Set lights to FLASHING GREEN
            lights_info: LightsRequest = LightsRequest()
            lights_info.red = 0
            lights_info.green = 255
            lights_info.blue = 0
            lights_info.flashing = True
            _ = self.send_lights_request(lights_info)
            rclpy.shutdown()

        # Ensure we've started receving rover coordinates
        if self._last_known_rover_coord is None:
            llogger.warning(
                "Can't start navigating until the GPS provides a coordinate. Returning early."
            )
            return

        # Ensure our current coordinate is updating
        time_since_rover_coord: Time = Time().from_msg(
            self._last_known_rover_coord.header.stamp
        )
        if self._sensor_data_timed_out(time_since_rover_coord):
            llogger.warning("gps coord hasn't updated; cannot navigate")
            return

        # when we haven't reached the coords yet (step 1), keep navigating to current coordinates (if not stopped to look at a tag)
        if not self.calculating_aruco_coord:
            # get current target coordinate from coordinate queue
            target_coord: GeoPoint = self._coordinate_path_queue[0]
            # calculate distance to target
            dist_to_target_coord_m = distance(
                [target_coord.latitude, target_coord.longitude],
                [
                    self._last_known_rover_coord.position.latitude,
                    self._last_known_rover_coord.position.longitude,
                ],
            ).meters

            # Check if destination is reacheed
            if dist_to_target_coord_m < MIN_GPS_DISTANCE:
                _ = self.get_logger().info("Reached destination coordinate!")
                # in GPS mode, we're only navigating to a coordinate.
                #
                # so we stop here and return!
                if (
                    self._param_value is not None
                    and self._param_value.mode == NavigationMode.GPS
                ):
                    # stop the coroutine
                    self._go_to_coordinate_cor = None
                    # stop the wheels
                    self.stop_wheels()
                    # WARNING: by making this `True`, we tell the function to
                    # stop itself from running the next time it's called
                    self.goal_reached = True
                    return

                # in ArUco or object detection mode, we want to move on to the next coordinate (provided by calculation)
                _ = self._coordinate_path_queue.pop(0)
            else:
                """Based on the target coordinate and the current coordinate of the Rover, we want to activate the
                go_to_coordinate async function to have the Rover start moving toward a coordinate. If the function is already
                running with the same coordinate, no need to stop the current function running."""
                # PID Controller function to send wheel speeds
                #
                # if we want to go somewhere else, we replace `self._go_to_coordinate_cor` with
                # another Coroutine.
                #
                # doing so will have the Rover moving while we make decisions.
                #
                # we can also cancel the Task
                if self._last_searched_coord != target_coord:
                    self._go_to_coordinate_cor = self._go_to_coordinate(
                        target_coord
                    )
                    self._last_searched_coord = target_coord
                pass

        match self._param_value.mode:
            case NavigationMode.GPS:
                pass

            case NavigationMode.ARUCO:
                # Do ArUco tracking stuff
                """
                random note: we're possibly definitely gonna need to reset the
                PID controller each time we traverse to a new coordinate
                """

                # Ensure we've at least seen the aruco marker once
                if self._curr_marker_transform is None:
                    llogger.debug(
                        f"Haven't seen the ArUco marker (id: {self._given_aruco_marker_id}) yet"
                    )
                    return

                # only use old info if ArUco marker has been seen in the last 2 seconds
                time_since_aruco_marker: Time = Time().from_msg(
                    self._curr_marker_transform.header.stamp
                )
                if self._sensor_data_timed_out(time_since_aruco_marker):
                    # if we've seen the marker this frame, increase the counter.
                    #
                    # when we've seen it enough times, we'll start moving
                    # toward it.
                    #
                    # this strategy reduces the likelihood of false positives
                    # impacting our plan/route
                    self._times_marker_seen += 1

                    # Calculate distance to marker
                    distance_to_marker = get_distance_to_marker(
                        self._curr_marker_transform
                    )

                    if distance_to_marker < MIN_ARUCO_DISTANCE:
                        # stop the coroutine
                        self._go_to_coordinate_cor = None
                        # stop the wheels
                        self.stop_wheels()
                        # WARNING: by making this `True`, we tell the function to
                        # stop itself from running the next time it's called
                        self.goal_reached = True
                        return
                    elif self._times_marker_seen > 20:
                        # WARNING! Won't this keep calculating the ArUco coordinate and appending to the coordinate queue after 20?

                        # check that we have recv'd any messages from the gps
                        if self._last_known_rover_coord is None:
                            llogger.warning(
                                "Found marker 20 times, but never got any Rover GPS coordinates! Returning early."
                            )
                            return

                        # Calculate the coordinate of the ArUco marker given the current Rover coordinate and the ArUco pose
                        aruco_coord = coordinate_from_aruco_pose(
                            self._last_known_rover_coord,
                            self._curr_marker_transform,
                        )
                        # Clear the coordinate queue
                        self._coordinate_path_queue.clear()
                        # Update rover coords queue and stop calculating aruco coordinate
                        self._coordinate_path_queue.insert(0, aruco_coord)
                        # Generate search coordinates around the ArUco coordinate
                        self._coordinate_path_queue.extend(
                            generate_similar_coordinates(aruco_coord, 10, 5)
                        )
                        # Append original GPS coordinate to the end of the queue
                        self._coordinate_path_queue.append(
                            self._param_value.coord
                        )
                        # NOTE: Should we reset the times marker seen here?
                        self.calculating_aruco_coord = False
                    else:
                        # Stop the coroutine (set it to none) so that we can sit still and check to see if we've seen the marker enough times
                        self._go_to_coordinate_cor = None
                        # Stop the wheels
                        self.stop_wheels()
                        self.calculating_aruco_coord = True

                else:  # Lost the marker, start traversing to previous coord
                    self._times_marker_seen = 0
                    self.calculating_aruco_coord = False

            # TODO: Get object detection, lul
            case NavigationMode.OBJECT_DETECTION:
                llogger.error("Object detection is currently unimplemented!")
                sys.exit(1)
        pass

    def stop_wheels(self):
        """
        Publish wheel speeds to stop the rover.
        """
        wheel_speeds: WheelsMessage = WheelsMessage()
        # wheel speed of none is represented by 126 for some reason
        wheel_speeds.left_wheels = 126
        wheel_speeds.right_wheels = 126
        self._wheels_publisher.publish(wheel_speeds)
        # log message that wheels have stopped
        _ = self.get_logger().info("Wheels stopped!")

    async def _go_to_coordinate(
        self,
        dest_coord: GeoPoint,
    ):
        """
        Given a GPS coordinate, navigate to it.
        This acts kind of like a ROS 2 action, where we can track it, cancel it, etc.
        This is a simple implementation that uses a PID controller to navigate to the coordinate, based
        on the rover's current position and the destination coordinate.
        """
        # Make sure that we are receiving rover imu and coordinates
        # TODO: Implement a version of this function that actually uses the IMU compass data. For now, we're just using the GPS data
        # if self._last_known_imu_data is None:
        #     llogger.warning("IMU data not yet received; cannot navigate to coordinate")
        #     return
        if self._last_known_rover_coord is None:
            llogger.warning(
                "GPS data not yet received; cannot navigate to coordinate"
            )
            return

        # get magnetometer info from IMU
        # compass_info: Vector3 = self._last_known_imu_data.compass # gives us x, y, z compass data

        # Utility functions
        def get_angle_to_dest(
            dest_coord: GeoPoint, current_coord: GeoPointStamped
        ) -> float:
            """Calculate the angle from the robot to the destination."""
            # TODO: Double check trigonometry
            unnormalized_angle = math.atan2(
                dest_coord.latitude - current_coord.position.latitude,
                dest_coord.longitude - current_coord.position.longitude,
            )
            # Normalize the angle to be between -pi and pi
            normalized_angle = (unnormalized_angle + math.pi) % (
                2 * math.pi
            ) - math.pi
            return normalized_angle

        # Start navigating to the coordinates using a PID controller
        # NOTE: I have a funny feeling this will send wheel speeds too fast
        # pk = proportional, pi = integral, pd = derivative
        # pk determines the speed of error correction, pi determines how much the rover will correct itself, and pd determines how quickly the rover will stop correcting itself
        pk, pi, pd = (
            self._param_value.pk,
            self._param_value.pi,
            self._param_value.pd,
        )  # NOTE: We may not want to use all of these
        target_value = 0  # We want the rover's angle to the destination be 0
        pid = PID(pk, pi, pd, setpoint=target_value)
        wheel_speeds: WheelsMessage = WheelsMessage()
        while True:
            angle_to_dest = get_angle_to_dest(
                dest_coord, self._last_known_rover_coord
            )
            pid_value = pid(angle_to_dest)

            wheel_speeds.right_wheels += pid_value
            wheel_speeds.left_wheels -= pid_value

            # Filter the wheel speeds
            # - make sure they aren't above max or min
            # max speed = 255, reverse at max speed = 0, middle/none = 126
            wheel_speeds.left_wheels = max(
                0, min(255, wheel_speeds.left_wheels)
            )
            wheel_speeds.right_wheels = max(
                0, min(255, wheel_speeds.right_wheels)
            )
            # Publish the wheel speeds
            self._wheels_publisher.publish(wheel_speeds)
            # Log the wheel speeds being sent
            _ = self.get_logger().info(
                f"Sending wheel speeds: left: {wheel_speeds.left_wheels}, right: {wheel_speeds.right_wheels}"
            )

    # Given a coordinate,
    async def _go_to_coordinate_with_reason(
        self, coord: GeoPoint, reason: GoToCoordinateReason
    ):
        """
        async so this acts kinda like a ros 2 action.

        u can track it, cancel it, etc.
        """

        # start moving
        # get stop distance for provided reason
        stop_distance: float
        match reason:
            case GoToCoordinateReason.GPS:
                stop_distance = MIN_GPS_DISTANCE
            case GoToCoordinateReason.ARUCO:
                stop_distance = MIN_ARUCO_DISTANCE

        # if we're within the required distance of the coordinate, we'll stop
        # automatically
        if self._near_coordinate(coord, stop_distance):
            return
        pass

    # Note: This is good, but idfk how to implement this with the structure that has already had a lot of work put into it. For the moment, ignoring this
    def _near_coordinate(
        self, target_coordinate: GeoPoint, distance_m: float
    ) -> bool:
        """
        Checks if the Rover is near a target coordinate, within the given
        distance (in meters).
        """
        # we may only check if we're near a coordinate if the gps has given us
        # the coordinate pair of the Rover
        if self._last_known_rover_coord is None:
            _ = self.get_logger().error(
                "The GPS hasn't yet sent a coordinate, so we don't know if we're\
                within distance of the given coordinate. Returning False."
            )
            return False

        # calculate the distance
        dist_to_target_coord_m = distance(
            [target_coordinate.latitude, target_coordinate.longitude],
            [
                self._last_known_rover_coord.position.latitude,
                self._last_known_rover_coord.position.longitude,
            ],
        ).meters

        # perform the actual check
        within_distance: bool = dist_to_target_coord_m < distance_m
        llogger.debug(
            f"Rover is near coordinate {target_coordinate}: {within_distance}"
        )
        return within_distance

    def _sensor_data_timed_out(self, sensor_data_timestamp: Time) -> bool:
        """
        Given a timestamp of sensor data (has to contain a `Header`), this
        method will check if it's too old to use ("has timed out").
        """

        current_time: int = self.get_clock().now().nanoseconds
        time_since_sensor_data: int = sensor_data_timestamp.nanoseconds

        # check if it's timed out
        timed_out: bool = (
            current_time - time_since_sensor_data
        ) > SENSOR_TIMEOUT_NS

        # if we've timed out on some data, print that out
        if timed_out:
            _ = self.get_logger().error("Sensor data timed out!")

        # return the boolean val
        return timed_out

    def gps_callback(self, msg: GpsMessage):
        self._last_known_rover_coord = msg.coords()

    def aruco_callback(self, msg: PoseStamped):
        self._curr_marker_transform = msg

    def imu_callback(self, msg: ImuMessage):
        self._last_known_imu_data = msg


def main(args: list[str] | None = None):
    """
    Initializes the Node using `rcl`.
    """
    rclpy.init(args=args)

    navigator_node: NavigatorNode = NavigatorNode()

    # spawn a task on the executor that continues running the Node until it's
    # destroyed.
    #
    # similar to the `tokio::spawn` syntax in Rust, but managed by `rcl` outside
    # the interpreter.
    #
    # slowdowns may occur, so consider benching anything questionable.
    rclpy.spin(navigator_node)

    # destroy the Node explicitly
    #
    # this is optional - otherwise, the garbage collector does it automatically
    # when it runs.
    navigator_node.destroy_node()
    rclpy.shutdown()


# runs the main function - Python doesn't do this automatically.
if __name__ == "__main__":
    main()
