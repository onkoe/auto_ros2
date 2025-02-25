from dataclasses import dataclass

import rclpy
from gps_msgs.msg import GPSFix as GpsFix
from loguru import logger as llogger
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.service import Service
from rclpy.subscription import Subscription
from sensor_msgs.msg import NavSatFix
from typing_extensions import override

from custom_interfaces.msg._gps_message import GpsMessage
from custom_interfaces.msg._wheels_message import WheelsMessage
from custom_interfaces.srv._lights import Lights
from custom_interfaces.srv._lights import Lights_Request as LightsRequest
from custom_interfaces.srv._lights import Lights_Response as LightsResponse

from .wheel_speeds import translate_u8

QOS_PROFILE = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass(kw_only=True)
class SoroBridge(Node):
    """Translates stuff from the simulator into the messages we use."""

    # motors... there are quite a few.
    #
    # these are directly from gazebo
    __left_front_wheel_motor: Publisher
    __left_middle_wheel_motor: Publisher
    __left_back_wheel_motor: Publisher
    __right_front_wheel_motor: Publisher
    __right_middle_wheel_motor: Publisher
    __right_back_wheel_motor: Publisher

    # interface speakers/listeners
    __lights_service: Service
    __wheels_subscriber: Subscription
    __gps_publisher: Publisher
    __imu_publisher: Publisher
    __depth_camera_publisher: Publisher
    __mono_camera_publisher: Publisher

    # fake sim interface speakers/listeners...
    #
    # these are used to publish messages like the `sensors_node` :D
    __sim_imu_subscriber: Subscription
    # __sim_gyro_subscriber: Subscription
    # __sim_accel_subscriber: Subscription
    # __sim_compass_subscriber: Subscription
    __sim_gps_subscriber: Subscription

    def __init__(self):
        # initialize the `Robot` superclass
        super().__init__("soro_bridge")

        self.__gps_publisher = self.create_publisher(
            GpsMessage, "/sensors/gps", QOS_PROFILE
        )
        self.__sim_gps_subscriber = self.create_subscription(
            GpsFix, "/sim/gps", self.sim_gps_callback, QOS_PROFILE
        )

        # react to controls
        self.__wheels_subscriber = self.create_subscription(
            WheelsMessage, "/control/wheels", self.wheels_callback, QOS_PROFILE
        )
        self.__lights_service = self.create_service(
            Lights, "/control/lights", self.lights_callback
        )

    @override
    def __hash__(self) -> int:
        return super().__hash__()

    def lights_callback(
        self, req: LightsRequest, resp: LightsResponse
    ) -> LightsResponse:
        llogger.debug(f"recv'd lights request! see: {req}")

        # change the color of the lights
        lights_value: int = 0x000000
        lights_value += req.red << 16
        lights_value += req.green << 8
        lights_value += req.blue
        # self.__led.set(lights_value) # FIXME

        return resp  # FIXME: add fields

    def wheels_callback(self, msg: WheelsMessage):
        llogger.debug(f"recv'd wheel speeds! see: {msg}")

        BASE_SPEED: float = 1.0
        left_wheel_speed: float = BASE_SPEED * translate_u8(msg.left_wheels)
        right_wheel_speed: float = BASE_SPEED * translate_u8(msg.right_wheels)

        # set left wheel speeds
        #
        # FIXME: this is wrong lol
        self.__left_front_wheel_motor.publish(left_wheel_speed)
        self.__left_middle_wheel_motor.publish(left_wheel_speed)
        self.__left_back_wheel_motor.publish(left_wheel_speed)

        # set right wheel speeds
        self.__right_front_wheel_motor.publish(right_wheel_speed)
        self.__right_middle_wheel_motor.publish(right_wheel_speed)
        self.__right_back_wheel_motor.publish(right_wheel_speed)
        pass

    def sim_gps_callback(self, msg: NavSatFix):
        """immediately publishes to the `/sensors/gps` topic after translating"""
        translated: GpsMessage = GpsMessage()

        translated.lat = msg.latitude
        translated.lon = msg.longitude
        translated.height = msg.altitude

        # using some default values for now.
        #
        # FIXME: iirc, there's a gz msg ty with real values for these
        translated.error_mm = 0.0
        translated.time_of_week = 0

        llogger.debug(f"publishing gps info: {translated}")
        self.__gps_publisher.publish(translated)


def main(args: list[str] | None = None):
    llogger.info("starting simulator driver...")
    rclpy.init(args=args)

    bridge_node: SoroBridge = SoroBridge()

    # spawn a task on the executor that continues running the Node until it's
    # destroyed.
    #
    # similar to the `tokio::spawn` syntax in Rust, but managed by `rcl` outside
    # the interpreter.
    #
    # slowdowns may occur, so consider benching anything questionable.
    rclpy.spin(bridge_node)

    # destroy the Node explicitly
    #
    # this is optional - otherwise, the garbage collector does it automatically
    # when it runs.
    bridge_node.destroy_node()
    rclpy.shutdown()


# runs the main function - Python doesn't do this automatically.
if __name__ == "__main__":
    main()
