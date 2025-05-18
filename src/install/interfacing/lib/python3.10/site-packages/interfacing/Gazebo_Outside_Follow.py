#!/usr/bin/env python3

# TurtleBot3 - Counterclockwise wall follower (hugging RIGHT wall @ 0.5m)

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class movement(Node):
    def __init__(self):
        super().__init__("outside_wall_follower")

        # Sub to LIDAR scan
        self.Lidardis = self.create_subscription(LaserScan, "scan", self.obstacle, 10)

        # Publisher to cmd_vel
        self.velocity_ = self.create_publisher(Twist, "cmd_vel", 10)

        # Timer for sending velocity commands
        self.seconds_per_clock = 0.1 
        self.timer = self.create_timer(self.seconds_per_clock, self.set_velocity)

        # Target distances
        self.desired_distance = 0.475 # target distance to wall (m) *Lowered to account for oversteer
        self.close_detect_range = 0.27 # range to keep wall far enough (m)
        self.detect_range = 0.52 # max range to consider wall detectable (m)
        self.detect_far = 2.5 # max range to consider wall detectable (m)
        self.lost_range = 3.5 # number to show no wall

        # Movement flags
        self.move_forward = True
        self.turn_left = False
        self.turn_right = False
        self.turn_right_fast = False

        # Sensor flags
        self.lwall = False
        self.rwall = False
        self.fwall = False
        self.rwall_far = False

    def obstacle(self, msg):
        x = len(msg.ranges) // 10  # Split scan into 10 sectors

        # Reset wall detections
        self.reset_wall_flags()

        # --- Left detection ---
        for j in range(x):
            i = j + 2 * x
            dist = msg.ranges[i]
            if msg.range_min < dist < self.desired_distance:
                self.lwall = True
                break

        # --- Right detection ---
        for j in range(x):
            i = j + 7 * x
            self.dist = msg.ranges[i]
            if msg.range_min < self.dist < self.desired_distance:
                self.rwall = True
                break
        
        # --- Right far detection ---
        for j in range(x):
            i = j + 8 * x
            self.dist = msg.ranges[i]
            if self.dist > self.lost_range:
                self.rwall_far = True
            elif msg.range_min < self.dist < self.close_detect_range:
                self.fwall = True
                break

        # --- Front detection ---
        for j in range(x):
            i = j + 0 * x
            dist = msg.ranges[i]
            if msg.range_min < dist < self.detect_range:
                self.fwall = True
                break
            # --- Front far detection ---
            if msg.ranges[i] > self.detect_far:
                self.fwall_far = True
                break

        for j in range(x):
            i = j + 9 * x
            dist = msg.ranges[i]
            if msg.range_min < dist < self.detect_range:
                self.fwall = True
                break
            # --- Front far detection ---
            if msg.ranges[i] > self.detect_far:
                self.fwall_far = True
                break

        self.navigate()

    def navigate(self):
        # Wall-following state logic (RIGHT wall hugger, counterclockwise)

        #Reset movement flags
        self.reset_flags()

        if self.fwall:
            # Wall close → turn left to avoid
            self.turn_left = True
            self.get_logger().info('Wall ahead or too close → Turning LEFT')
        elif self.rwall_far and self.fwall_far:
            self.turn_right_fast = True
            self.get_logger().info('Right wall far → Going around corner')
        elif self.rwall_far:
            #implemented for initialization
            self.start_wall = True
            self.get_logger().info('Right wall far → Starting to follow')
        elif self.rwall:
            # Wall on right → move forward
            self.move_forward = True
            self.get_logger().info('Following wall. Wall is ' + str(self.dist) + 'm away')
        else:
            # No right wall → turn right to reacquire
            self.turn_right = True
            self.get_logger().info('Right wall lost → Turning RIGHT')


    def set_velocity(self):
        msg = Twist()

        if self.move_forward:
            msg.linear.x = 0.35
            msg.angular.z = -0.015 # small correction to keep wall on right
        elif self.turn_right_fast:
            msg.linear.x = 0.15 # move foward a bit for wall detection
            msg.angular.z = -0.25 # sharper turn right for corner
        elif self.turn_left:
            msg.linear.x = 0.005
            msg.angular.z = 0.15  # sharper left turn
        elif self.turn_right:
            msg.linear.x = 0.15
            msg.angular.z = -0.15  # right turn
        elif self.start_wall:
            msg.linear.x = 0.25
            msg.angular.z = 0.05 # small correction to keep wall on right

        self.velocity_.publish(msg)
    
    def reset_flags(self):
        self.move_forward = False
        self.turn_left = False
        self.turn_right = False
        self.turn_right_fast = False
        self.start_wall = False
    
    def reset_wall_flags(self):
        self.lwall = False
        self.rwall = False
        self.fwall = False
        self.rwall_far = False
        self.fwall_far = False

def main(args=None):
    rclpy.init(args=args)
    node = movement()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
