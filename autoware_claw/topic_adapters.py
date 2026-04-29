"""Message conversion between Nav2/ROSClaw and Autoware formats."""

from __future__ import annotations

import math

from autoware_control_msgs.msg import Control, Lateral, Longitudinal
from builtin_interfaces.msg import Time


def twist_to_control(
    linear_x: float,
    angular_z: float,
    wheelbase: float,
    stamp: Time | None = None,
) -> Control:
    """Convert Nav2-style Twist (linear.x, angular.z) to Autoware Control msg.

    Uses bicycle model: steering = atan(angular_z * wheelbase / linear_x)
    """
    msg = Control()
    if stamp:
        msg.stamp = stamp

    # Lateral: steering tire angle
    if abs(linear_x) > 1e-6:
        steering = math.atan(angular_z * wheelbase / linear_x)
    elif abs(angular_z) > 1e-6:
        steering = math.copysign(math.pi / 2.0, angular_z)
    else:
        steering = 0.0

    msg.lateral = Lateral()
    msg.lateral.steering_tire_angle = float(steering)
    msg.lateral.steering_tire_rotation_rate = 0.0

    # Longitudinal: velocity + acceleration
    msg.longitudinal = Longitudinal()
    msg.longitudinal.velocity = float(linear_x)
    msg.longitudinal.acceleration = 0.0
    msg.longitudinal.jerk = 0.0

    return msg


def make_control_msg(
    steering_tire_angle_rad: float,
    velocity_mps: float,
    acceleration_mps2: float = 0.0,
    stamp: Time | None = None,
) -> Control:
    """Build an Autoware Control message from explicit values."""
    msg = Control()
    if stamp:
        msg.stamp = stamp

    msg.lateral = Lateral()
    msg.lateral.steering_tire_angle = float(steering_tire_angle_rad)
    msg.lateral.steering_tire_rotation_rate = 0.0

    msg.longitudinal = Longitudinal()
    msg.longitudinal.velocity = float(velocity_mps)
    msg.longitudinal.acceleration = float(acceleration_mps2)
    msg.longitudinal.jerk = 0.0

    return msg
