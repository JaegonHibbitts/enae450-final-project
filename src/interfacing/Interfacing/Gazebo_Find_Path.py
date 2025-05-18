from time import sleep

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np

class ExplorerBot(Node):
    def __init__(self):
        super().__init__('explorer_bot')

        # Subscribers & Publishers
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for sending velocity commands
        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.set_velocity)

        # Parameters
        self.obs_thresh = 0.5       # Collision bubble threshold (m)
        self.large_dist_thresh = 2.0 # Large space threshold (m)
        self.long_period = 2.0       # Move forward duration (large space)
        self.short_period = 0.5      # Move forward duration (small space)

        # States
        self.turning_angle = 0.0
        self.forward_duration = 0.0
        self.front_clear = True

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[ranges == 0.0] = msg.range_max  # Replace invalid 0s

        # === Obstacle avoidance (front sector −30° to +30°) ===
        sector_size = len(ranges) // 12  # 360° / 12 = 30° sectors
        front_ranges = np.concatenate([ranges[-sector_size:], ranges[:sector_size]])

        if np.any(front_ranges < self.obs_thresh):
            self.front_clear = False
            self.get_logger().info('Obstacle detected ahead — stopping!')
            self.turning_angle = 0.5  # Turn slightly right to dodge
            self.forward_duration = 0.0
            return
        else:
            self.front_clear = True

        # === Open-space seeking ===
        max_dist = np.max(ranges)
        max_index = np.argmax(ranges)

        angle_per_index = msg.angle_increment  # radians
        angle_to_target = msg.angle_min + max_index * angle_per_index

        self.get_logger().info(f'Max open distance: {max_dist:.2f} m at angle {np.degrees(angle_to_target):.1f}°')

        # Determine turn direction and magnitude
        self.turning_angle = np.clip(angle_to_target, -1.0, 1.0)  # limit turning speed

        # Decide forward duration
        if max_dist > self.large_dist_thresh:
            self.forward_duration = self.long_period
        else:
            self.forward_duration = self.short_period

    def set_velocity(self):
        msg = Twist()

        # Priority: obstacle avoidance first
        if not self.front_clear:
            msg.linear.x = 0.0
            msg.angular.z = self.turning_angle
        else:
            if abs(self.turning_angle) > 0.2:
                msg.linear.x = 0.0
                msg.angular.z = self.turning_angle
                self.get_logger().info(f'Turning towards open space: {self.turning_angle:.2f} rad/s')
            elif self.forward_duration > 0.0:
                msg.linear.x = 0.2
                msg.angular.z = 0.0
                self.forward_duration -= self.timer_period
                self.get_logger().info('Moving forward')
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerBot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
