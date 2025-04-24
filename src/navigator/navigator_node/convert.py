import math

from sensor_msgs.msg import Imu


def compass_degrees_z(compass_msg: Imu):
    """
    Grabs the compass value, in degrees, pointing on the Z axis.
    """

    # get compass in deg from quat.
    #
    # we're gonna assume +z is forward, so it's just typical 3d space
    qx: float = compass_msg.orientation.x
    qy: float = compass_msg.orientation.y
    qz: float = compass_msg.orientation.z
    qw: float = compass_msg.orientation.w

    # this is the rotation on the z-axis
    #
    # from wikipedia:
    #
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    yaw: float = math.atan2(
        2.0 * (qw * qz + qx * qy), 1.0 - (2.0 * (qy * qy + qz * qz))
    )

    # convert it into degrees
    compass_degrees: float = (math.degrees(yaw) + 360) % 360
    return compass_degrees
