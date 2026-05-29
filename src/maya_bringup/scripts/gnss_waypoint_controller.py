#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

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

        self.target_latitude = self.get_parameter('target_latitude').value
        self.target_longitude = self.get_parameter('target_longitude').value
        self.goal_radius_m = self.get_parameter('goal_radius_m').value
        self.max_linear_speed = self.get_parameter('max_linear_speed_mps').value
        self.max_angular_speed = self.get_parameter('max_angular_speed_radps').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance_rad').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.gnss_timeout_s = self.get_parameter('gnss_timeout_s').value
        self.odom_timeout_s = self.get_parameter('odom_timeout_s').value
        self.allow_odom_yaw_as_enu = self.get_parameter('allow_odom_yaw_as_enu').value

        self.latest_gnss: NavSatFix | None = None
        self.latest_odom: Odometry | None = None
        self.latest_gnss_time = 0.0
        self.latest_odom_time = 0.0
        self.yaw_offset = self._initial_yaw_offset()
        self.stopped_at_goal = False
        self.last_report_time = 0.0

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10
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

        if self.target_latitude == 0.0 and self.target_longitude == 0.0:
            self._stop_throttled('No target_latitude/target_longitude configured.')
            return

        now = time.monotonic()
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

        distance, desired_yaw, east, north = distance_and_bearing_enu(
            self.latest_gnss.latitude,
            self.latest_gnss.longitude,
            self.target_latitude,
            self.target_longitude,
        )
        if distance <= self.goal_radius_m:
            self._publish_stop()
            self.stopped_at_goal = True
            self.get_logger().info(
                f'Goal reached: distance={distance:.2f}m east={east:.2f}m north={north:.2f}m'
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
                'GNSS nav '
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

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def main() -> None:
    rclpy.init()
    node = GnssWaypointController()
    try:
        rclpy.spin(node)
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
