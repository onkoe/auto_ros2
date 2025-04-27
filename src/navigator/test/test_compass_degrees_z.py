"""
Tests that the compass conversion function works as intended.
"""

from math import cos, sin
from math import pi as PI

from geometry_msgs.msg import Quaternion
from navigator_node.convert import (
    compass_degrees_z,
)
from pytest import approx
from sensor_msgs.msg import Imu

X_AXIS: float = 0.0
Y_AXIS: float = 0.0
Z_AXIS: float = 1.0


def _make_imu_msg_helper(angle_radians: float) -> Imu:
    """Makes an IMU message with orientation rotating around the Z axis..."""

    msg: Imu = Imu()
    orientation: Quaternion = Quaternion()
    orientation.x = X_AXIS * sin(angle_radians / 2.0)
    orientation.y = Y_AXIS * sin(angle_radians / 2.0)
    orientation.z = Z_AXIS * sin(angle_radians / 2.0)
    orientation.w = cos(angle_radians / 2.0)
    msg.orientation = orientation

    return msg


def test_90_deg():
    msg: Imu = _make_imu_msg_helper(PI / 2)
    result: float = compass_degrees_z(msg)
    assert result == approx(90.0)


def test_negative_90_deg():
    msg: Imu = _make_imu_msg_helper(-PI / 2)
    result: float = compass_degrees_z(msg)
    assert result == approx(270.0)


def test_32_deg():
    # note: this is the opposite conversion, so should work fine
    _32_deg: float = (PI * 32) / 180

    msg: Imu = _make_imu_msg_helper(_32_deg)
    result: float = compass_degrees_z(msg)
    assert result == approx(32.0)


def test_full_imu_0_deg():
    """Tests behavior with a 'full' IMU message."""
    _32_deg: float = (PI * 32) / 180

    msg: Imu = Imu()

    # note: these values are from the simulator when the rover was facing
    # forward
    orientation: Quaternion = Quaternion()
    orientation.x = 2.956974537272026e-10
    orientation.y = 8.848844337308426e-11
    orientation.z = 1.539359383401774e-16
    orientation.w = 1.0
    msg.orientation = orientation

    result: float = compass_degrees_z(msg)
    assert result == approx(0.0)
