"""
This module assists in estimating the location of goals as WSG84 coordinate
pairs.

Doing so lets us assign goals coordinates instead of trying to adjust using local
translations to the physical Rover.
"""

import sys

from custom_interfaces.msg import ArucoPoseMessage
from geographic_msgs.msg import GeoPoint
from loguru import logger as llogger


def coordinate_from_aruco_pose(
    current_location: GeoPoint, pose: ArucoPoseMessage
) -> GeoPoint:
    """
    Given the Rover's current location and the ArUco marker's current pose,
    this function calculates an approximate coordinate for the marker.

    These coordinates allow the Navigator to start moving toward an ArUco
    marker.
    """
    llogger.error("coordinate estimation is unimplemented!")
    sys.exit(1)
