import asyncio
import sys
from dataclasses import dataclass

import rclpy
from geographic_msgs.msg import GeoPoint, GeoPointStamped, GeoPose
from geometry_msgs.msg import PoseStamped
from loguru import logger as llogger
from nav2_simple_commander.robot_navigator import BasicNavigator
from rcl_interfaces.msg import ParameterType
from rclpy.client import Client
from rclpy.node import (
    Node,
    ParameterDescriptor,
)
from rclpy.publisher import Publisher
from rclpy.qos import QoSPresetProfiles, QoSProfile
from rclpy.subscription import Subscription
from sensor_msgs.msg import Imu, NavSatFix
from typing_extensions import override

from custom_interfaces.msg import (
    WheelsMessage,
)
from custom_interfaces.srv import Lights
from custom_interfaces.srv._lights import (
    Lights_Request as LightsRequest,
)
from custom_interfaces.srv._lights import (
    Lights_Response as LightsResponse,
)

from .convert import compass_degrees_z

# from custom_interfaces.msg import ArMessage as ArucoMessage
from .coords import dist_m_between_coords
from .types import (
    DEFAULT_PID_DERIVATIVE_GAIN,
    DEFAULT_PID_INTEGRAL_GAIN,
    DEFAULT_PID_PROPORTIONAL_GAIN,
    NavigationMode,
    NavigationParameters,
)

## how long we'll keep the data (DDS)
QOS_PROFILE: QoSProfile = QoSPresetProfiles.SENSOR_DATA.value
"""
The amount of time until we ignore a message from a sensor.

Using this allows us to ignore old messages, only using new ones.

Measured in nanoseconds.
"""

## minimum distance we need to be within from the coordinate
MIN_GPS_DISTANCE: float = 3.0  # meters
MIN_ARUCO_DISTANCE: float = 2.0  # meters


@dataclass(kw_only=True)
class NavigatorNode(Node):
    _wheels_publisher: Publisher
    """Publish wheels speeds to move the Rover."""
    _gps_subscription: Subscription
    _aruco_subscription: Subscription
    _imu_subscription: Subscription
    """Grabs information from the IMU, which includes compass, acceleration, and orientation."""
    _lights_client: Client
    """Service client for the Lights node."""

    nav_parameters: NavigationParameters
    _given_aruco_marker_id: int | None = None
    """
    The ID of the ArUco marker we're looking for.

    We don't use this directly - it's only to make error messages better.
    """

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
    _last_known_compass_direction: float | None = None
    """The direction in which the compass was last known to be pointing."""

    _curr_marker_transform: PoseStamped | None = None
    _times_marker_seen: int = 0

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

        _ = self.get_logger().info("Starting Navigator...")

        # declare each parameter as required by ROS 2...
        #
        # coord
        latitude_desc: ParameterDescriptor = ParameterDescriptor()
        latitude_desc.description = "latitude for coordinate we'll nav to"
        latitude_desc.type = ParameterType.PARAMETER_DOUBLE
        _ = self.declare_parameter(name="latitude", descriptor=latitude_desc)
        longitude_desc: ParameterDescriptor = ParameterDescriptor()
        longitude_desc.description = "longitude for coordinate we'll nav to"
        longitude_desc.type = ParameterType.PARAMETER_DOUBLE
        _ = self.declare_parameter(name="longitude", descriptor=longitude_desc)

        # mode
        mode_desc: ParameterDescriptor = ParameterDescriptor()
        mode_desc.description = "the thing we're doing right now. i.e. to do \
            aruco tracking, use NavigationMode.ARUCO"
        mode_desc.type = ParameterType.PARAMETER_INTEGER
        _ = self.declare_parameter(name="mode", descriptor=mode_desc)

        # pid controller controls
        pid_desc: ParameterDescriptor = ParameterDescriptor()
        pid_desc.type = ParameterType.PARAMETER_DOUBLE
        _ = self.declare_parameter(
            name="pk", value=DEFAULT_PID_PROPORTIONAL_GAIN, descriptor=pid_desc
        )
        _ = self.declare_parameter(
            name="pi", value=DEFAULT_PID_INTEGRAL_GAIN, descriptor=pid_desc
        )
        _ = self.declare_parameter(
            name="pd", value=DEFAULT_PID_DERIVATIVE_GAIN, descriptor=pid_desc
        )
        _ = self.get_logger().debug("declared all parameters.")

        # try to grab the instructions we're given over parameters.
        #
        # if none is given, this is `None`, and will cause the program to go
        # down
        latitude: float | None = self.get_parameter("latitude").value
        longitude: float | None = self.get_parameter("longitude").value
        if latitude is None:
            _ = self.get_logger().error(
                "The `latitude` parameter is not set! The navigator will now exit."
            )
            sys.exit(1)
        if longitude is None:
            _ = self.get_logger().error(
                "The `longitude` parameter is not set! The navigator will now exit."
            )
            sys.exit(1)

        # construct coordinate from lat + long
        coord: GeoPoint = GeoPoint()
        coord.latitude = latitude
        coord.longitude = longitude
        coord.altitude = (
            0.0  # TODO(bray): literally just take this from the rust library lol
        )

        mode_int: int | None = self.get_parameter("mode").value
        if mode_int is None:
            _ = self.get_logger().error(
                "The `coord` parameter is not set! The navigator will now exit."
            )
            sys.exit(1)

        pk: float | None = self.get_parameter("pk").value
        pi: float | None = self.get_parameter("pi").value
        pd: float | None = self.get_parameter("pd").value
        if pd is None or pi is None or pk is None:
            _ = self.get_logger().error(
                "PID parameters were unset, but this should result in a default. \
                This is unexpected behavior. The navigator will now exit."
            )
            sys.exit(1)

        # construct the parameters
        self.nav_parameters = NavigationParameters(
            coord=coord,
            mode=NavigationMode(mode_int),  # FIXME: do a safety check first
            pk=pk,
            pi=pi,
            pd=pd,
        )
        _ = self.get_logger().debug("constructed all parameters.")

        # create a service client for lights node
        self._lights_client = self.create_client(
            srv_type=Lights,
            srv_name="/control/lights",
        )

        # wait for the lights service to come up
        llogger.debug("Attempting to connect to lights service...")
        _ = self._lights_client.wait_for_service()
        llogger.debug("Lights service is up!")

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
            topic="/control/wheels",
            qos_profile=QOS_PROFILE,
        )

        # if in aruco mode, create a subscriber for aruco tracking
        if self.nav_parameters.mode == NavigationMode.ARUCO:
            self._aruco_subscription = self.create_subscription(
                msg_type=PoseStamped,
                topic="/marker_pose",
                callback=self.aruco_callback,
                qos_profile=QOS_PROFILE,
            )

        # connect to our sensors using subscriptions
        self._gps_subscription = self.create_subscription(
            msg_type=NavSatFix,
            topic="/sensors/gps",
            callback=self.gps_callback,
            qos_profile=QOS_PROFILE,
        )
        self._imu_subscription = self.create_subscription(
            msg_type=Imu,
            topic="/sensors/imu",
            callback=self.imu_callback,
            qos_profile=QOS_PROFILE,
        )

        # Add the given GPS coordinate to the coordinate queue
        #
        # TODO(bray): we still want to perform a search, but we should try
        #             doing so in a functional way...
        """
        self._coordinate_path_queue = []
        self._coordinate_path_queue.append(self.nav_parameters.coord)
        # Calculate and append search coordinates for GPS coord
        self._coordinate_path_queue.extend(
            generate_similar_coordinates(self.nav_parameters.coord, 10, 5)
        )  # takes source coordinate, radius in meters, and a number of points to generate
        """

    # all ROS 2 nodes must be hashable!
    @override
    def __hash__(self) -> int:
        return super().__hash__()

    # Function to send lights request given a LightsRequest class instance
    def send_lights_request(self, lights_info: LightsRequest) -> LightsResponse | None:
        """
        Send a request to the lights service to turn the lights red.
        """
        lights_future = self._lights_client.call_async(lights_info)
        rclpy.spin_until_future_complete(self, lights_future)
        return lights_future.result()

    async def navigator(self):
        """
        This is an async function driven by the `asyncio` runtime created in
        the `main()` function at the bottom of this file.

        It's async such that any async functions we `await` in here have access
        to the runtime. (Otherwise, they wouldn't be able to run async.)

        Note that this function will stop running after it reaches its goal.
        """

        await self._wait_for_sensors()

        # step 1: nav to coordinate.
        #
        # no matter what, we're going to navigate to a coordinate, as our
        # other tasks also require going to one.
        #
        # ...grab the coordinate we want to head to
        target_coord: GeoPoint = self.nav_parameters.coord
        llogger.info(f"Step 1: go to coordinate! target: {target_coord}")

        dist_to_target_coord_m: float = dist_m_between_coords(
            self._last_known_rover_coord.position,  # pyright: ignore[reportOptionalMemberAccess]
            target_coord,
        )

        # if we're not at the coordinate, go there!
        if dist_to_target_coord_m > MIN_GPS_DISTANCE:
            llogger.debug(
                f"Not yet at target coordinate! Navigating... (coord: {target_coord})"
            )
            await self._go_to_coordinate(target_coord)

        # step 2: handle aruco/object detection, if we need to
        match self.nav_parameters.mode:
            case NavigationMode.GPS:
                pass
            case NavigationMode.ARUCO:
                _ = self.get_logger().info(
                    "At requested coordinate! Now searching for ArUco marker."
                )
                await self.handle_aruco_navigation()
            case NavigationMode.OBJECT_DETECTION:
                _ = self.get_logger().info(
                    "At requested coordinate! Now performing object detection search."
                )
                _ = self.get_logger().fatal("Object detection is unimplemented.")
                sys.exit(1)  # because it's unimplemented.
                # TODO(bray): remove the above.

        # step 3: we've reached our target! flash lights and return...
        _ = self.get_logger().info("Goal reached!")
        self._flash_lights()
        shutdown(self)
        return

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
        # convert our func's given `GeoPoint` into a destination `GeoPose`
        dest: GeoPose = GeoPose()
        dest.position.latitude = dest_coord.latitude
        dest.position.longitude = dest_coord.longitude
        dest.position.altitude = dest_coord.altitude

        # create a `BasicNavigator` from `nav2_simple_commander`
        basic_nav: BasicNavigator = BasicNavigator()

        # start navigating.
        #
        # note that the `go_to_pose` function returns a bool indicating whether
        # the destination was "accepted" by Nav2.
        #
        # if it wasn't accepted, we'll wait a bit and try again.
        while True:
            if basic_nav.goToPose(dest):
                # move on to managing the basic nav
                break
            else:
                # wait a bit to try again
                llogger.error(
                    "Failed to navigate to destination: Nav2 rejected the target. Retrying in 5 seconds..."
                )
                await asyncio.sleep(5.0)
            pass
        pass

        # great, we've started navigating!
        #
        # let's sit here and manage it until it's done.
        while True:
            # when we've completed the navigation task, break out
            if basic_nav.isTaskComplete():
                break
            pass

            # otherwise, we're still navigating. wait a moment to check again
            await asyncio.sleep(1.0)
        pass

        # alright, we've finished coordinate navigation! let's tell the user...
        llogger.info("Navigated to target successfully!")

    # TODO(log): implement
    async def handle_aruco_navigation(self):
        """
        Performs the second step of the Navigator node (if requested), which is
        ArUco navigation.
        """

        # Ensure we've at least seen the aruco marker once
        if self._curr_marker_transform is None:
            llogger.debug(
                f"Haven't seen the ArUco marker (id: {self._given_aruco_marker_id}) yet. Performing search algorithm..."
            )
            _ = self.get_logger().fatal("searching for aruco is unimplemented!")
            shutdown(self)
            sys.exit(1)

    def _flash_lights(self):
        """
        we have reached our goal, so we are going to ask the `lights_node` to
        flash the lights for us, then shut down.

        flashing the lights is a requirement from
        """
        # Set lights to FLASHING GREEN
        lights_info: LightsRequest = LightsRequest()
        lights_info.red = 0
        lights_info.green = 255
        lights_info.blue = 0
        lights_info.flashing = True
        _ = self.send_lights_request(lights_info)
        return

    async def _wait_for_sensors(self):
        """
        This function will return after we get sensor information from all
        required sensors.

        Await it to idle until we have sensor data.
        """
        # Ensure we've started receving rover coordinates
        while self._last_known_rover_coord is None:
            llogger.warning(
                "Can't start navigating until the GPS provides a coordinate. Waiting to begin..."
            )
            await asyncio.sleep(0.5)
        pass

        # wait for the GPS and IMU to do something before we begin navigating
        while self._last_known_compass_direction is None:
            _ = self.get_logger().warn("Sensor data isn't up yet. Waiting to navigate.")
            _ = self.get_logger().debug(
                f"sensor data: compass: {self._last_known_compass_direction}, gps: {self._last_known_rover_coord}"
            )
            await asyncio.sleep(0.25)
        pass

    def gps_callback(self, msg: NavSatFix):
        # move its header over
        gp_stamped: GeoPointStamped = GeoPointStamped()
        gp_stamped.header = msg.header

        # and stick the coord stuff in a geopoint
        gp: GeoPoint = GeoPoint()
        gp.latitude = msg.latitude
        gp.longitude = msg.longitude
        gp.altitude = msg.altitude

        # mv the geopoint into that parent type
        gp_stamped.position = gp

        # finally, set that type on the navigator class
        self._last_known_rover_coord = gp_stamped

    def aruco_callback(self, msg: PoseStamped):
        self._curr_marker_transform = msg

    def imu_callback(self, msg: Imu):
        """
        Converts the nonsense IMU message into compass deg, then saves it.
        """
        compass_deg_z: float = compass_degrees_z(msg)
        self._last_known_compass_direction = compass_deg_z

    pass


pass

"""
HEY!

The following is related to starting the `asyncio` runtime and creating tasks
to handle the ROS 2 node + `rclpy` connection.

You don't need to touch it when adjusting behavior...
"""


def main(args: list[str] | None = None):
    """
    Starts the Navigator node.
    """
    rclpy.init(args=args)
    navigator_node: NavigatorNode = NavigatorNode()

    # spin the ros 2 node and run our logic... at the same time!
    #
    # for more info, see: https://github.com/m2-farzan/ros2-asyncio
    future = asyncio.wait([ros_spin(navigator_node), navigator_node.navigator()])
    _ = asyncio.get_event_loop().run_until_complete(future)

    # destroy the Node explicitly
    #
    # this is optional - otherwise, the garbage collector does it automatically
    # when it runs.
    navigator_node.destroy_node()
    rclpy.shutdown()


def shutdown(n: NavigatorNode):
    """Immediately turns off the Navigator."""
    n.stop_wheels()
    rclpy.shutdown()


async def ros_spin(node: Node):
    """
    Spins the given ROS 2 node. This allows it to connect to other DDS
    entities.
    """

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(1e-4)

    llogger.error("`rclpy.ok()` no longer True. the node is no longer spinning.")
