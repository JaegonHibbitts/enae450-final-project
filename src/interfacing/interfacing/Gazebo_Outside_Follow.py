#!/usr/bin/env python3

# TurtleBot3 - Counterclockwise wall follower (hugging RIGHT wall @ 0.5m)

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class movement(Node):
    def __init__(self):
        super().__init__("wall_follower")

        # Sub to LIDAR scan
        self.Lidardis = self.create_subscription(LaserScan, "scan", self.obstacle, 10)

        # Publisher to cmd_vel
        self.velocity_ = self.create_publisher(Twist, "cmd_vel", 10)

        # Timer for sending velocity commands
        self.seconds_per_clock = 0.1  # 10 Hz update
        self.timer = self.create_timer(self.seconds_per_clock, self.set_velocity)

        # Target distances
        self.desired_distance = 0.5   # target distance to wall (m)
        self.detect_range = 0.6      # max range to consider wall detectable (m)

        # Movement flags
        self.move_forward = True
        self.turn_left = False
        self.turn_right = False

        # Sensor flags
        self.lwall = False
        self.rwall = False
        self.fwall = False

    def obstacle(self, msg):
        x = len(msg.ranges) // 10  # Split scan into 10 sectors

        # Reset wall detections
        self.lwall = False
        self.rwall = False
        self.fwall = False

        # --- Left detection ---
        for j in range(x):
            i = j + 2 * x
            dist = msg.ranges[i]
            if msg.range_min < dist < self.detect_range:
                self.lwall = True
                break

        # --- Right detection ---
        for j in range(x):
            i = j + 7 * x
            self.dist = msg.ranges[i]
            if msg.range_min < self.dist < self.detect_range:
                self.rwall = True
                break

        # --- Front detection ---
        for j in range(x):
            i = j + 0 * x
            dist = msg.ranges[i]
            if msg.range_min < dist < self.detect_range:
                self.fwall = True
                break

        for j in range(x):
            i = j + 9 * x
            dist = msg.ranges[i]
            if msg.range_min < dist < self.detect_range:
                self.fwall = True
                break

        self.navigate()

    def navigate(self):
        # Wall-following state logic (RIGHT wall hugger, counterclockwise)

        if not self.rwall:
            # No right wall → turn left to reacquire
            self.move_forward = False
            self.turn_left = False
            self.turn_right = True
            self.get_logger().info('Right wall lost → Turning RIGHT')
        elif self.fwall:
            # Wall in front → turn left to avoid
            self.move_forward = False
            self.turn_left = True
            self.turn_right = False
            self.get_logger().info('Wall ahead → Turning LEFT')
        else:
            # Wall on right → move forward
            self.move_forward = True
            self.turn_left = False
            self.turn_right = False
            self.get_logger().info('Following wall. Wall is ' + str(self.dist) + 'm away')

    def set_velocity(self):
        msg = Twist()

        if self.move_forward:
            msg.linear.x = 0.3
            msg.angular.z = 0.0
        elif self.turn_left:
            msg.linear.x = 0.15
            msg.angular.z = 0.15  # gentle left turn
        elif self.turn_right:
            msg.linear.x = 0.15
            msg.angular.z = -0.15  # gentle right turn

        self.velocity_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = movement()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
