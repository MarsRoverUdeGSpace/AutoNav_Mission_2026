#!/usr/bin/env python3

import math


METERS_PER_DEGREE_LAT = 111111.0


def wrap_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compass_deg_to_enu_yaw(compass_heading_deg: float) -> float:
    """Convert compass heading degrees, north=0 clockwise, to ENU yaw radians."""
    return wrap_pi((math.pi * 0.5) - math.radians(compass_heading_deg))


def enu_delta_meters(
    current_latitude: float,
    current_longitude: float,
    target_latitude: float,
    target_longitude: float,
) -> tuple[float, float]:
    """Return target offset from current position as (east_m, north_m)."""
    latitude_rad = math.radians(current_latitude)
    north = (target_latitude - current_latitude) * METERS_PER_DEGREE_LAT
    east = (
        (target_longitude - current_longitude)
        * METERS_PER_DEGREE_LAT
        * math.cos(latitude_rad)
    )
    return east, north


def distance_and_bearing_enu(
    current_latitude: float,
    current_longitude: float,
    target_latitude: float,
    target_longitude: float,
) -> tuple[float, float, float, float]:
    """Return distance, bearing, east offset, and north offset to a WGS84 target."""
    east, north = enu_delta_meters(
        current_latitude,
        current_longitude,
        target_latitude,
        target_longitude,
    )
    return math.hypot(east, north), math.atan2(north, east), east, north


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract planar yaw from a geometry_msgs-style quaternion."""
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)
