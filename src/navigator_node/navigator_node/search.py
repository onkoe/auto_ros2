"""
Implementation of the target search algorithm.
"""

from geographic_msgs.msg import GeoPoint
from loguru import logger as llogger


async def search_for_target(lead_coordinates: list[GeoPoint]):
    """
    Moves the Rover to each potential target location we've collected. The info
    obtained from performing this search isn't managed by this function.

    Instead, other ROS 2 nodes will tell the Navigator that it has found
    something, in which case it can cancel this async function.

    IMPORTANT: Ensure that you don't re-use this list (or that it has been
    deep-copied)! We might modify it.
    """
    llogger.debug("starting search for a list of leads...")

    # iterate over each "potential location" (where a target would be located)
    for potential_location in lead_coordinates:
        llogger.debug(
            f"checking location at ({potential_location.latitude} lat, {potential_location.longitude} lon)..."
        )
        pass

    pass
