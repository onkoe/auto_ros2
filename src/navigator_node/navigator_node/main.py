import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription

from std_msgs.msg import String

from typing import List
from dataclasses import dataclass

## how long we'll keep the data (DDS).
QUEUE_SIZE: int = 10


@dataclass(kw_only=True)
class NavigatorNode(Node):
    ## to make us a subscriber
    _subscription: Subscription
    ## how many msgs we've recv'd
    _msg_ct: int = 0

    def __init__(self):
        """
        Creates a new `NavigatorNode`.
        """

        # give the node a name.
        #
        # this syntax is a little ambiguous, but just know that it's calling
        # the superclass constructor with otherwise default values.
        super().__init__("navigator_node")

        # create the publisher
        self._subscription = self.create_subscription(
            msg_type=String,
            topic="topic",
            callback=self._msg_callback,
            qos_profile=QUEUE_SIZE,
        )

    def _msg_callback(self, msg: String):
        """
        When we get a message, this function runs.

        It'll just print out some info about what we got.
        """
        self._msg_ct += 1
        self.get_logger().info(
            f"Got a message! msg_ct: {self._msg_ct}, msg: {msg.data}"
        )

    def __hash__(self) -> int:
        return super().__hash__()


def main(args: List[str] | None = None):
    """
    Initializes the Node using `rcl`.
    """
    rclpy.init(args=args)

    navgiator_node: NavigatorNode = NavigatorNode()

    # spawn a task on the executor that continues running the Node until it's
    # destroyed.
    #
    # similar to the `tokio::spawn` syntax in Rust, but managed by `rcl` outside
    # the interpreter.
    #
    # slowdowns may occur, so consider benching anything questionable.
    rclpy.spin(navgiator_node)

    # destroy the Node explicitly
    #
    # this is optional - otherwise, the garbage collector does it automatically
    # when it runs.
    navgiator_node.destroy_node()
    rclpy.shutdown()


# runs the main function - Python doesn't do this automatically.
if __name__ == "__main__":
    main()
