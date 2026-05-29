#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8

from gnss_waypoint_common import (
    compass_deg_to_enu_yaw,
    distance_and_bearing_enu,
    wrap_pi,
    yaw_from_quaternion,
)


class GnssWaypointController(Node):
    """Conservative direct-control GNSS waypoint driver for first field tests."""

    def __init__(self) -> None:
        super().__init__('gnss_waypoint_controller')

        self.declare_parameter('target_latitude', 0.0)
        self.declare_parameter('target_longitude', 0.0)
        self.declare_parameter('waypoints', '')
        self.declare_parameter('target_altitude', 0.0)
        self.declare_parameter('goal_radius_m', 3.0)
        self.declare_parameter('max_linear_speed_mps', 0.15)
        self.declare_parameter('max_angular_speed_radps', 0.35)
        self.declare_parameter('yaw_tolerance_rad', 0.35)
        self.declare_parameter('linear_kp', 0.25)
        self.declare_parameter('angular_kp', 0.8)
        self.declare_parameter('gnss_timeout_s', 2.0)
        self.declare_parameter('odom_timeout_s', 1.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('initial_compass_heading_deg', float('nan'))
        self.declare_parameter('yaw_offset_rad', float('nan'))
        self.declare_parameter('allow_odom_yaw_as_enu', False)
        self.declare_parameter('gnss_topic', '/sensors/gnss/fix')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('mode_topic', '/mode')
        self.declare_parameter('goal_reached_mode_value', 2)
        self.declare_parameter('normal_mode_value', 1)
        self.declare_parameter('goal_reached_mode_hold_s', 5.0)

        self.waypoints = self._load_waypoints()
        self.active_waypoint_index = 0
        self.goal_radius_m = self.get_parameter('goal_radius_m').value
        self.max_linear_speed = self.get_parameter('max_linear_speed_mps').value
        self.max_angular_speed = self.get_parameter('max_angular_speed_radps').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance_rad').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.gnss_timeout_s = self.get_parameter('gnss_timeout_s').value
        self.odom_timeout_s = self.get_parameter('odom_timeout_s').value
        self.allow_odom_yaw_as_enu = self.get_parameter('allow_odom_yaw_as_enu').value
        self.goal_reached_mode_value = int(
            self.get_parameter('goal_reached_mode_value').value
        )
        self.normal_mode_value = int(self.get_parameter('normal_mode_value').value)
        self.goal_reached_mode_hold_s = max(
            0.0, float(self.get_parameter('goal_reached_mode_hold_s').value)
        )

        self.latest_gnss: NavSatFix | None = None
        self.latest_odom: Odometry | None = None
        self.latest_gnss_time = 0.0
        self.latest_odom_time = 0.0
        self.yaw_offset = self._initial_yaw_offset()
        self.stopped_at_goal = False
        self.goal_hold_until = 0.0
        self.mode_is_goal_reached = False
        self.last_report_time = 0.0

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10
        )
        self.mode_pub = self.create_publisher(
            UInt8, self.get_parameter('mode_topic').value, 10
        )
        self.create_subscription(
            NavSatFix,
            self.get_parameter('gnss_topic').value,
            self._on_gnss,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10
        )

        rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.create_timer(1.0 / rate_hz, self._control_tick)

        self.get_logger().warn(
            'GNSS direct controller armed only after target/heading validation. '
            'Use low speed and keep manual stop ready.'
        )
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} ordered GNSS waypoint(s).'
        )

    def _load_waypoints(self) -> list[tuple[float, float]]:
        configured_waypoints = str(self.get_parameter('waypoints').value).strip()
        if configured_waypoints:
            waypoints: list[tuple[float, float]] = []
            for index, item in enumerate(configured_waypoints.split(';'), start=1):
                fields = [field.strip() for field in item.split(',')]
                if len(fields) != 2:
                    raise ValueError(
                        f'Waypoint {index} must be "latitude,longitude"; got "{item}".'
                    )
                latitude = float(fields[0])
                longitude = float(fields[1])
                self._validate_waypoint(latitude, longitude, index)
                waypoints.append((latitude, longitude))
            return waypoints

        latitude = float(self.get_parameter('target_latitude').value)
        longitude = float(self.get_parameter('target_longitude').value)
        if latitude == 0.0 and longitude == 0.0:
            return []
        self._validate_waypoint(latitude, longitude, 1)
        return [(latitude, longitude)]

    def _validate_waypoint(self, latitude: float, longitude: float, index: int) -> None:
        if not (math.isfinite(latitude) and -90.0 <= latitude <= 90.0):
            raise ValueError(f'Waypoint {index} latitude is invalid: {latitude}')
        if not (math.isfinite(longitude) and -180.0 <= longitude <= 180.0):
            raise ValueError(f'Waypoint {index} longitude is invalid: {longitude}')

    def _initial_yaw_offset(self) -> float | None:
        configured_offset = float(self.get_parameter('yaw_offset_rad').value)
        if math.isfinite(configured_offset):
            return wrap_pi(configured_offset)

        compass_heading = float(self.get_parameter('initial_compass_heading_deg').value)
        if math.isfinite(compass_heading):
            # Offset is finalized when the first odom yaw arrives.
            return None

        if self.allow_odom_yaw_as_enu:
            self.get_logger().warn('Using raw odom yaw as ENU yaw by operator request.')
            return 0.0

        return None

    def _on_gnss(self, msg: NavSatFix) -> None:
        self.latest_gnss = msg
        self.latest_gnss_time = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_odom_time = time.monotonic()
        if self.yaw_offset is None:
            compass_heading = float(
                self.get_parameter('initial_compass_heading_deg').value
            )
            if math.isfinite(compass_heading):
                odom_yaw = self._odom_yaw(msg)
                self.yaw_offset = wrap_pi(compass_deg_to_enu_yaw(compass_heading) - odom_yaw)
                self.get_logger().info(
                    f'Computed yaw_offset_rad={self.yaw_offset:.3f} '
                    f'from initial_compass_heading_deg={compass_heading:.1f}'
                )

    def _control_tick(self) -> None:
        if self.stopped_at_goal:
            return

        now = time.monotonic()
        if self.goal_hold_until > 0.0:
            self._publish_stop()
            if now < self.goal_hold_until:
                return
            self._publish_mode(self.normal_mode_value)
            self.mode_is_goal_reached = False
            self.goal_hold_until = 0.0
            self.active_waypoint_index += 1
            if self.active_waypoint_index >= len(self.waypoints):
                self.stopped_at_goal = True
                self.get_logger().info('Final GNSS waypoint complete.')
                return
            self.get_logger().info(
                f'Advancing to GNSS waypoint {self.active_waypoint_index + 1}/'
                f'{len(self.waypoints)}.'
            )

        if not self.waypoints:
            self._stop_throttled('No GNSS waypoint configured.')
            return

        if self.latest_gnss is None or (now - self.latest_gnss_time) > self.gnss_timeout_s:
            self._stop_throttled('GNSS stale or missing.')
            return
        if self.latest_odom is None or (now - self.latest_odom_time) > self.odom_timeout_s:
            self._stop_throttled('Odometry stale or missing.')
            return
        if self.yaw_offset is None:
            self._stop_throttled(
                'No Earth yaw reference. Set initial_compass_heading_deg, '
                'yaw_offset_rad, or allow_odom_yaw_as_enu:=true.'
            )
            return
        if not self._gnss_healthy(self.latest_gnss):
            self._stop_throttled('GNSS status/covariance is not healthy enough.')
            return

        target_latitude, target_longitude = self.waypoints[self.active_waypoint_index]
        distance, desired_yaw, east, north = distance_and_bearing_enu(
            self.latest_gnss.latitude,
            self.latest_gnss.longitude,
            target_latitude,
            target_longitude,
        )
        if distance <= self.goal_radius_m:
            self._publish_stop()
            self._publish_mode(self.goal_reached_mode_value)
            self.mode_is_goal_reached = True
            self.goal_hold_until = now + self.goal_reached_mode_hold_s
            self.get_logger().info(
                f'GNSS waypoint {self.active_waypoint_index + 1}/{len(self.waypoints)} '
                f'reached: distance={distance:.2f}m east={east:.2f}m north={north:.2f}m. '
                f'Publishing mode={self.goal_reached_mode_value} for '
                f'{self.goal_reached_mode_hold_s:.1f}s.'
            )
            return

        odom_yaw = self._odom_yaw(self.latest_odom)
        earth_yaw = wrap_pi(odom_yaw + self.yaw_offset)
        yaw_error = wrap_pi(desired_yaw - earth_yaw)

        cmd = Twist()
        cmd.angular.z = self._clamp(
            self.angular_kp * yaw_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        if abs(yaw_error) <= self.yaw_tolerance:
            cmd.linear.x = min(self.max_linear_speed, self.linear_kp * distance)
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)
        if now - self.last_report_time > 1.0:
            self.last_report_time = now
            self.get_logger().info(
                f'GNSS nav waypoint={self.active_waypoint_index + 1}/{len(self.waypoints)} '
                f'distance={distance:.2f}m east={east:.2f}m north={north:.2f}m '
                f'desired_yaw={desired_yaw:.2f} earth_yaw={earth_yaw:.2f} '
                f'yaw_error={yaw_error:.2f} cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})'
            )

    def _gnss_healthy(self, msg: NavSatFix) -> bool:
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return False
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return False
        covariance = msg.position_covariance
        if covariance[0] > 100.0 or covariance[4] > 100.0:
            return False
        return True

    def _odom_yaw(self, msg: Odometry) -> float:
        q = msg.pose.pose.orientation
        return yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _stop_throttled(self, reason: str) -> None:
        self._publish_stop()
        now = time.monotonic()
        if now - self.last_report_time > 2.0:
            self.last_report_time = now
            self.get_logger().warn(reason)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _publish_mode(self, value: int) -> None:
        msg = UInt8()
        msg.data = int(value)
        self.mode_pub.publish(msg)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def main() -> None:
    rclpy.init()
    node = GnssWaypointController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
            if node.mode_is_goal_reached:
                node._publish_mode(node.normal_mode_value)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
