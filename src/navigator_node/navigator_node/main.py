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

import sys
import time
from dataclasses import dataclass
from enum import Enum

import rclpy
from custom_interfaces.msg import ArMessage as ArucoMessage
from custom_interfaces.msg import GpsMessage, WheelsMessage
from custom_interfaces.srv import LightsRequest, LightsResponse
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from geopy.distance import distance
from loguru import logger as llogger
from rclpy.client import Client
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription
from rclpy.task import Future
from rclpy.timer import Timer

from .coords import coordinate_from_aruco_pose

## how long we'll keep the data (DDS).
QUEUE_SIZE: int = 10

## minimum distance we need to be within from the coordinate
MIN_GPS_DISTANCE: float = 3  # meters
MIN_ARUCO_DISTANCE: float = 2  # meters


class NavigationMode(Enum):
    GPS = 0
    ARUCO = 1
    OBJECT_DETECTION = 2


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


@dataclass(kw_only=True)
class NavigatorNode(Node):
    _wheels_publisher: Publisher
    """Publish wheels speeds to move the Rover."""
    _gps_subscription: Subscription
    _aruco_subscription: Subscription
    ## service client for lights node
    _lights_client: Client

    _navigator_callback_timer: Timer
    """The timer that runs the main `navigator` logic."""

    _lights_request_future: Future
    """A `Future` that we await to change the light color."""

    _param_value: NavigationParameters | None

    _times_marker_seen: int = 0

    # these let us check if we've reached stuff
    coords_reached: bool = False
    """When this is true, we've reached the coordinate near the goal."""
    goal_reached: bool = False
    """When true, we've reached the object itself."""

    # info about where the rover/target is
    _last_known_rover_coord: GeoPoint | None = None
    """The coordinate last recv'd from the GPS."""
    _last_known_marker_coord: GeoPoint | None = None
    """Coordinate pair where the target was last known to be located."""

    _curr_marker_transform: PoseStamped | None = None

    # coordinate queue
    _coord_queue: list[GeoPoint] = []

    # current gps attribute

    def __init__(self):
        """
        Creates a new `NavigatorNode`.
        """
        super().__init__("Navigator Node")
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
        lights_info: LightsRequest = LightsRequest.Request()
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
                msg_type=ArucoMessage,  # TODO: change to wherever the ar message type is
                topic="/aruco",  # TODO: change to whatever the markers topic is
                callback=self.navigator,
                qos_profile=QUEUE_SIZE,
            )

        # create a subscriber for GPS sensor
        # create the publisher
        self._gps_subscription = self.create_subscription(
            msg_type=GpsMessage,
            topic="/sensors/gps",
            callback=self.navigator,
            qos_profile=QUEUE_SIZE,
        )

        # Add the given GPS coordinate to the coordinate queue
        self._coord_queue.append(self._param_value.coord)
        # Calculate and append search coordinates for GPS coord
        self.append_search_coords(self._coord_queue)
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

    # FIXME: distance and angle to marker should never be passed
    # FIXME: wheel speeds can be determined for any mode, not just marker
    # TODO: use `GeoPoint` type?
    def get_wheel_speeds(self, distance_to_marker, angle_to_marker):
        pass

    # TODO: change msg type to whatever the gps message type is
    def navigator(self, _msg: str):
        """
        TODO: docs...

        Driven by a `rclpy::Timer`.
        """

        # When the last time, we updated GPS
        # - If took too long, freak out

        #

        # If goal is reached, turn the node off
        if self.goal_reached:
            # Log message that goal was reached
            _ = self.get_logger().info("Goal reached!")
            #           # Set lights to FLASHING GREEN
            lights_info: LightsRequest = LightsRequest.Request()
            lights_info.red = 0
            lights_info.green = 255
            lights_info.blue = 0
            lights_info.flashing = True
            _ = self.send_lights_request(lights_info)
            rclpy.shutdown()

        if self._param_value is None:
            llogger.error("Called, but no parameters given.")
            sys.exit(1)

        # when we haven't reached the coords yet (step 1), keep goin
        if not self.coords_reached:
            # we may only continue if the GPS has provided a coordinate for the
            # Rover
            if self._last_known_rover_coord is None:
                llogger.warning(
                    "Can't start navigating until the GPS provides a coordinate. Returning early."
                )
                return

            target_coord = self._coord_queue.pop(0)
            # calculate distance to target
            dist_to_target_coord_m = distance(
                [target_coord.latitude, target_coord.longitude],
                [
                    self._last_known_rover_coord.latitude,
                    self._last_known_rover_coord.longitude,
                ],
            ).meters

            # Check if destination is reacheed
            if dist_to_target_coord_m < MIN_GPS_DISTANCE:
                _ = self.get_logger().info("Reached destination coordinate!")
                self.coords_reached = True

                # in GPS mode, we're only navigating to a coordinate.
                #
                # so we stop here and return!
                #
                # FIXME: returning won't stop the timer. so do that first.
                if self._param_value.mode == NavigationMode.GPS:
                    self.goal_reached = True
                    return  # Return here right?

            else:
                # Calculate wheel speeds to send
                #
                # FIXME: write `wheel_speeds` for any mode lol
                # wheel_speeds = self.get_wheel_speeds(
                #     distance_to_coords, angle_to_coords
                # )
                pass

        match self._param_value.mode:
            case NavigationMode.GPS:
                pass

            case NavigationMode.ARUCO:
                # Do ArUco tracking stuff
                #
                # Should probably make sure this information isn't old first
                """
                (noir jazz [muted trumpet solo *sounds like weeping {like the sound of dying love}*] plays in the background)
                |_t: T|::<T> -> () { _t; }
                ROVERS INTERNAL THOUGHTS: [coords]
                ROVERS INTERNAL THOUGHTS: [coords, search1, search2, search3, search4]
                * It's going through these...*
                * Arrives at original coord   *
                ROVERS INTERNAL THOUGHTS: [search1, search2, search3, search4]
                * Going through these... *
                */ a doc comment that doesnt need to be in a multi-line comment */
                ROVER [speaking]: Oh! We found a tag!
                ROVERS INTERNAL THOUGHTS :[tag, tagsearch1, tagsearch2, tagsearch3, tagsearch4] TAG TAKES PRIORITY
                *dances epic style*

                random note: we're possibly definitely gonna need to reset the
                PID controller each time we traverse to a new coordinate
                """

                # only use old info if ArUco marker has been seen in the last 2 seconds
                if self._curr_marker_transform.header.stamp - time.time() < 2.0:
                    # if we've seen the marker this frame, increase the counter.
                    #
                    # when we've seen it enough times, we'll start moving
                    # toward it.
                    #
                    # this strategy reduces the liklihood of false positives
                    # impacting our plan/route
                    self._times_marker_seen += 1

                    # Calculate distance and angle to marker
                    distance_to_marker, angle_to_marker = (
                        get_dist_angle_to_marker(self._curr_marker_transform)
                    )

                    if distance_to_marker < MIN_ARUCO_DISTANCE:
                        self.goal_reached = True
                        return  # Return here right?
                    elif self._times_marker_seen > 20:
                        # check that we have recv'd any messages from the gps
                        if self._last_known_rover_coord is None:
                            llogger.warning(
                                "Found marker 20 times, but never got any Rover GPS coordinates! Returning early."
                            )
                            return

                        self._param_value.coord = coordinate_from_aruco_pose(
                            self._last_known_rover_coord,
                            self._curr_marker_transform,
                        )
                    else:
                        wheel_speeds = 0

            # TODO: Get object detection, lul
            case NavigationMode.OBJECT_DETECTION:
                llogger.error("Object detection is currently unimplemented!")
                sys.exit(1)
        pass

    def gps_callback(self, msg: GpsMessage):
        self._last_known_rover_coord = msg.coords()

    def aruco_callback(self, msg: PoseStamped):
        self._curr_marker_transform = msg


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
