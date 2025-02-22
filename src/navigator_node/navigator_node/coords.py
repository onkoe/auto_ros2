"""
This module assists in estimating the location of goals as WSG84 coordinate
pairs.

Doing so lets us assign goals coordinates instead of trying to adjust using local
translations to the physical Rover.
"""

import sys

from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import Point, PoseStamped
from geopy.distance import distance
from loguru import logger as llogger

from custom_interfaces.msg import ArucoPoseMessage


def coordinate_from_aruco_pose(
    _current_location: GeoPointStamped, _pose: ArucoPoseMessage
) -> GeoPoint:
    """
    Given the Rover's current location and the ArUco marker's current pose,
    this function calculates an approximate coordinate for the marker.

    These coordinates allow the Navigator to start moving toward an ArUco
    marker.
    """
    llogger.error("coordinate estimation is unimplemented!")
    sys.exit(1)

def get_distance_to_marker(current_location: GeoPointStamped, marker: PoseStamped) -> float:
    """
    Given the pose information for an ArUco marker, calculate the distance to the marker
    """
    marker_position: Point = marker.pose.point
    # get the current position of the rover
    rover_position: GeoPointStamped = current_location.position
    # calculate the distance to the marker
    dist_to_marker = distance(
        [marker_position.y, marker_position.x],
        [rover_position.latitude, rover_position.longitude],
    ).meters
    return dist_to_marker
