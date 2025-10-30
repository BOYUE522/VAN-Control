import math
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import tf2_ros

from car_interfaces.msg import SurroundingInfoInterface


def wrap_angle(angle: float) -> float:
    """Wrap angle to [0, 2*pi)."""
    angle = math.fmod(angle, 2.0 * math.pi)
    if angle < 0.0:
        angle += 2.0 * math.pi
    return angle


class SurroundingInfoFusion(Node):
    """Fusion node that aggregates multiple LaserScan topics into a 240-dim distance vector."""

    def __init__(self) -> None:
        super().__init__('uw_surrounding_info')

        # Parameters
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('scan_topics', [
            '/front_lidar/scan',
            '/rl_lidar/scan',
            '/rr_lidar/scan',
        ])
        self.declare_parameter('max_distance', 30.0)
        self.declare_parameter('num_bins', 240)
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('path_length', 100)
        self.declare_parameter('path_step', 0.3)

        self.base_frame: str = self.get_parameter('base_frame').get_parameter_value().string_value
        self.scan_topics: List[str] = [
            s.strip() for s in self.get_parameter('scan_topics').get_parameter_value().string_array_value
        ]
        self.max_distance: float = self.get_parameter('max_distance').value
        self.num_bins: int = int(self.get_parameter('num_bins').value)
        self.publish_frequency: float = self.get_parameter('publish_frequency').value
        self.path_length: int = int(self.get_parameter('path_length').value)
        self.path_step: float = self.get_parameter('path_step').value

        if self.num_bins <= 0:
            self.get_logger().warn('num_bins must be positive, defaulting to 240')
            self.num_bins = 240

        if not self.scan_topics:
            self.get_logger().error('scan_topics parameter is empty. Node will not publish any data.')

        # Prepare static path placeholder (straight line along +X)
        path_points = [(self.path_step * (i + 1), 0.0) for i in range(self.path_length)]
        self.path_rfu_flat: List[float] = [coord for point in path_points for coord in point]

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.time.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage for latest scans
        self.latest_scans: Dict[str, LaserScan] = {}

        # Subscribers
        for topic in self.scan_topics:
            self.create_subscription(LaserScan, topic, self._scan_callback, 10)
            self.latest_scans[topic] = None  # type: ignore[assignment]

        # Timer for publishing
        period = 1.0 / self.publish_frequency if self.publish_frequency > 0 else 0.1
        self.publisher = self.create_publisher(SurroundingInfoInterface, 'surrounding_info_data', 10)
        self.timer = self.create_timer(period, self._publish_callback)

        self.get_logger().info(f'SurroundingInfoFusion started. base_frame={self.base_frame}, '
                               f'scan_topics={self.scan_topics}')

    def _scan_callback(self, msg: LaserScan) -> None:
        self.latest_scans[msg._topic_name] = msg  # type: ignore[attr-defined]

    def _lookup_yaw_offset(self, frame_id: str) -> Optional[float]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                frame_id,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warning_once(f'Unable to lookup transform from {self.base_frame} to {frame_id}: {exc}')
            return None

        quat = transform.transform.rotation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw

    def _publish_callback(self) -> None:
        start_time = self.get_clock().now()

        bins = np.full(self.num_bins, self.max_distance, dtype=np.float32)

        valid_scan_found = False

        for topic, scan in self.latest_scans.items():
            if scan is None:
                continue

            yaw_offset = self._lookup_yaw_offset(scan.header.frame_id)
            if yaw_offset is None:
                continue

            range_min = max(scan.range_min, 0.0)
            range_max = scan.range_max if scan.range_max > 0.0 else self.max_distance
            angle = scan.angle_min

            for idx, distance in enumerate(scan.ranges):
                if not math.isfinite(distance):
                    angle += scan.angle_increment
                    continue
                if distance < range_min or distance > range_max:
                    angle += scan.angle_increment
                    continue

                final_angle = wrap_angle(angle + yaw_offset)
                bin_idx = int((final_angle / (2.0 * math.pi)) * self.num_bins)
                bin_idx = min(max(bin_idx, 0), self.num_bins - 1)

                capped_distance = min(distance, self.max_distance)
                if capped_distance < bins[bin_idx]:
                    bins[bin_idx] = capped_distance

                angle += scan.angle_increment

            valid_scan_found = True

        if not valid_scan_found:
            self.get_logger().warn_once('No valid LaserScan data received yet; skipping publish.')
            return

        normalized = (bins / self.max_distance).tolist()

        msg = SurroundingInfoInterface()
        msg.timestamp = start_time.nanoseconds * 1e-9
        msg.surroundinginfo = normalized
        msg.path_rfu = self.path_rfu_flat
        msg.turn_signals = 0.0
        msg.process_time = (self.get_clock().now() - start_time).nanoseconds * 1e-9
        msg.carlength = 0.0
        msg.carwidth = 0.0
        msg.carheight = 0.0
        msg.carspeed = 0.0
        msg.steerangle = 0.0
        msg.throttle_percentage = 0
        msg.braking_percentage = 0
        msg.braketq = 0.0
        msg.car_run_mode = 1  # default to autonomous; user may override later
        msg.gearpos = 3  # default forward
        msg.error_yaw = 0.0
        msg.error_distance = 0.0

        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SurroundingInfoFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
