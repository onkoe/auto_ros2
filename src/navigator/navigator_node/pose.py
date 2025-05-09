from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped
from loguru import logger as llogger
from std_msgs.msg import Header


def geopoint_to_pose(converted_geopoint: Point, time: Time) -> PoseStamped:
    """
    Converts a GeoPoint-derived offset into a `geometry_msgs::PoseStamped`.

    This conversion is necessary for giving goals to Nav2.
    """

    # make a new stamped pose.
    #
    # it's required for nav2
    llogger.debug("creating empty `geometry_msgs::PoseStamped`...")
    pose: PoseStamped = PoseStamped()
    llogger.debug("`geometry_msgs::PoseStamped` created!")

    # add the header
    h: Header = Header()
    llogger.debug("adding map...")
    h.frame_id = "map"
    llogger.debug("adding time...")
    h.stamp = time
    pose.header = h
    llogger.debug("map and time added!")

    # set our position correctly lol
    llogger.debug("adding position data...")
    pose.pose.position.x = converted_geopoint.x
    pose.pose.position.y = converted_geopoint.y
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0  # TODO(bray): test this lol
    llogger.debug("position data added successfully!")

    return pose
