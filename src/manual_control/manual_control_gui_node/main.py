import signal
import sys
from dataclasses import dataclass, field

import pygame
import rclpy
from geometry_msgs.msg import Twist
from loguru import logger as llogger
from pygame.joystick import JoystickType
from rclpy.node import Node, Publisher
from rclpy.qos import QoSPresetProfiles, QoSProfile
from rclpy.timer import Timer
from rclpy.utilities import try_shutdown
from typing_extensions import override

# define some constants for our topics
_CMD_VEL_TOPIC: str = "/cmd_vel"

# we'll also be using a reliable qos for sending inputs
_RELIABLE_QOS: QoSProfile = QoSPresetProfiles.SERVICES_DEFAULT.value

_DEFAULT_SCREEN_SIZE: tuple[int, int] = (480, 240)

# joystick direction magic numbers. probably wrong-ish bc they're from copilot lol
_LT_AXIS: int = 2
_RT_AXIS: int = 5
_TURN_AXIS = 0


@dataclass(kw_only=True)
class ManualControlGuiNode(Node):
    # ros stuff
    _pygame_update_timer: Timer
    _pygame_render_timer: Timer
    _cmd_vel_pub: Publisher

    # pygame stuff
    _screen: pygame.Surface
    _font: pygame.font.Font
    _joystick: JoystickType | None = None
    _axes: list[float] = field(default_factory=lambda: [])

    def __init__(self):
        super().__init__("manual_control_gui_node")

        # set up control topic
        self._cmd_vel_pub = self.create_publisher(Twist, _CMD_VEL_TOPIC, _RELIABLE_QOS)

        # initialize pygame stuff
        _ = pygame.init()
        pygame.font.init()

        # make a window
        self._screen = pygame.display.set_mode(_DEFAULT_SCREEN_SIZE)
        pygame.display.set_caption("ROS 2 - Manual Control")
        self._font = pygame.font.SysFont(None, 32)

        # check if we've got a controller
        self._rescan_controllers()

        # update pygame 60 times per second
        self._pygame_update_timer = self.create_timer(1.0 / 60.0, self._update_pygame)
        self._pygame_render_timer = self.create_timer(1.0 / 60.0, self._draw_pygame)

        # draw the first frame!
        self._draw_pygame()

    def _rescan_controllers(self):
        # update our joystick handle
        if pygame.joystick.get_count() > 0:
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
            self._axes = [0.0] * self._joystick.get_numaxes()
        else:
            if self._joystick is not None:
                self._joystick.quit()
            self._joystick = None
            self._axes = []

    def _draw_pygame(self):
        # set a quick and dirty controller status message
        msg: str = "no controller detected"
        if self._joystick is not None:
            msg = "controller detected"

        _ = self._screen.fill((30, 30, 30))
        text_surf = self._font.render(msg, True, (220, 220, 220))

        # draw the text at the center of the screen
        center: tuple[int, int] = (
            int(self._screen.get_size()[0] / 2),
            int(self._screen.get_size()[1] / 2),
        )
        text_rect = text_surf.get_rect(center=center)
        _ = self._screen.blit(text_surf, text_rect)
        pygame.display.flip()

    def _update_pygame(self) -> None:
        # update our controller state
        self._rescan_controllers()

        # check for new controller events
        for event in pygame.event.get():
            # quit the... game
            if event.type == pygame.QUIT:
                llogger.info("pygame exit requested! killing the ros node...")
                _ = rclpy.try_shutdown()

                llogger.info("stopping pygame...")
                pygame.quit()

                llogger.info("goodbye!")
                sys.exit(0)

            # controller movement
            if event.type == pygame.JOYAXISMOTION:
                self._axes[event.axis] = event.value  # pyright: ignore[reportAny]
                if event.axis in (_LT_AXIS, _RT_AXIS, _TURN_AXIS):  # pyright: ignore[reportAny]
                    self._publish_cmd_vel()

    def _publish_cmd_vel(self):
        if not self._axes:
            return
        try:
            speed: float = self._axes[_RT_AXIS] - self._axes[_LT_AXIS]
            turn: float = self._axes[_TURN_AXIS]
        except IndexError:
            llogger.error("index error! can't grab controller axis...")
            return

        msg = Twist()
        msg.linear.x = max(-1.0, min(1.0, speed))
        msg.angular.z = max(-1.0, min(1.0, turn))
        self._cmd_vel_pub.publish(msg)

    @override
    def __hash__(self) -> int:
        return super().__hash__()


def exit_handler(_handler: int, _):
    _ = try_shutdown()
    sys.exit(0)


def main(args: list[str] | None = None):
    rclpy.init(args=args)
    node: ManualControlGuiNode = ManualControlGuiNode()

    _ = signal.signal(signal.SIGINT, exit_handler)
    _ = signal.signal(signal.SIGTERM, exit_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        _ = try_shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
