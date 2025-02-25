"""
This module assists in estimating the location of goals as WSG84 coordinate
pairs.

Doing so lets us assign goals coordinates instead of trying to adjust using local
translations to the physical Rover.
"""

import sys
from math import sqrt

from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import Point, PoseStamped
from loguru import logger as llogger


def coordinate_from_aruco_pose(
    _current_location: GeoPointStamped, _pose: PoseStamped
) -> GeoPoint:
    """
    Given the Rover's current location and the ArUco marker's current pose,
    this function calculates an approximate coordinate for the marker.

    These coordinates allow the Navigator to start moving toward an ArUco
    marker.

    If we can get the current heading/bearing of the rover from compass info,
    we can use that and the distance to the marker to calculate the estimated
    coordinate.
    """
    marker_position: Point = _pose.pose.position
    marker_orientation = _pose.pose.orientation

    llogger.error("coordinate estimation is unimplemented!")
    sys.exit(1)


def get_distance_to_marker(marker: PoseStamped) -> float:
    """
    Given the pose information for an ArUco marker relative to the rover,
    calculate the distance to the marker
    """
    marker_position: Point = marker.pose.point
    distance_x: float = marker_position.x  # forward/backward distance
    distance_y: float = marker_position.y  # left/right distance

    # now we need to calculate the distance to the marker by taking the hypotenuse
    distance_to_marker: float = sqrt(distance_x**2 + distance_y**2)
    return distance_to_marker
