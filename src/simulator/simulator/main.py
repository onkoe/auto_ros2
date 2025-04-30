import signal
import sys
from dataclasses import dataclass

import rclpy
from loguru import logger as llogger
from rclpy.node import Node
from rclpy.qos import (
    QoSPresetProfiles,
    QoSProfile,
)
from rclpy.service import Service
from rclpy.utilities import try_shutdown
from typing_extensions import override

from custom_interfaces.srv._lights import Lights
from custom_interfaces.srv._lights import Lights_Request as LightsRequest
from custom_interfaces.srv._lights import Lights_Response as LightsResponse

QOS_PROFILE: QoSProfile = QoSPresetProfiles.SYSTEM_DEFAULT.value


@dataclass(kw_only=True)
class SoroBridge(Node):
    """Translates stuff from the simulator into the messages we use."""

    # interface speakers/listeners
    __lights_service: Service

    def __init__(self):
        # initialize the `Robot` superclass
        super().__init__("soro_bridge")

        # react to controls
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


def main(args: list[str] | None = None):
    # exit gracefully when Ctrl^C'd
    def exit_handler(_handler: int, _):
        llogger.info("Asked to shutdown! Doing so gracefully...")
        _ = try_shutdown()
        sys.exit(0)

    _ = signal.signal(signal.SIGINT, exit_handler)
    _ = signal.signal(signal.SIGTERM, exit_handler)

    llogger.info("Starting simulator driver...")
    rclpy.init(args=args)

    bridge_node: SoroBridge = SoroBridge()
    llogger.info("Simulator has been initialized.")

    # spawn a task on the executor that continues running the Node until it's
    # destroyed.
    #
    # similar to the `tokio::spawn` syntax in Rust, but managed by `rcl` outside
    # the interpreter.
    #
    # slowdowns may occur, so consider benching anything questionable.
    try:
        rclpy.spin(bridge_node)
        llogger.info("Simulator is spinning...")
    except KeyboardInterrupt:
        pass
    finally:
        # destroy the Node explicitly
        #
        # this is optional - otherwise, the garbage collector does it automatically
        # when it runs.
        bridge_node.destroy_node()
        _ = try_shutdown()


# runs the main function - Python doesn't do this automatically.
if __name__ == "__main__":
    main()
