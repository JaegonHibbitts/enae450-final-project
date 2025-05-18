#!/usr/bin/env python3

# TurtleBot3 - Clockwise wall follower (hugging LEFT wall @ 0.5m)

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Movement(Node):
    def __init__(self):
        super().__init__("inside_wall_follower")

        #parameters
        self.declare_parameter('direction')
        self.declare_parameter('desired_distance')
        self.get_params()
        
        # Sub to LIDAR scan
        self.Lidardis = self.create_subscription(LaserScan, "scan", self.obstacle, 10)

        # Publisher to cmd_vel
        self.velocity_ = self.create_publisher(Twist, "cmd_vel", 10)

        # Timer for sending velocity commands
        self.seconds_per_clock = 0.1  # 10 Hz update
        self.timer = self.create_timer(self.seconds_per_clock, self.set_velocity)

        # Target distances
        #self.desired_distance = 0.5   # target distance to wall (m)
        self.detect_range = 0.8     # range to keep wall far enough (m)
        self.far = 2               # max range to consider wall detectable (m)

        # Movement flags
        self.move_forward_R = False
        self.move_forward_L = False
        self.move_forward = False
        self.turn_left = False
        self.turn_right = False

        # Sensor flags
        self.lwall = False
        self.rwall = False
        self.fwall = False

        # Proportional controller gain
        self.kp = 1.0

    def get_params(self):
        # Get parameter values
        self.direction = self.get_parameter('direction').get_parameter_value().string_value
        self.desired_distance = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.get_logger().info(f"Parameters loaded - Direction: {self.direction}, Distance: {self.desired_distance}")
    
    def obstacle(self, msg):
        x = len(msg.ranges) // 10  # Split scan into 8 sectors

        # Reset wall detections
        self.lwall = False
        self.rwall = False
        self.fwall = False
        self.rwall_far = False
        self.lwall_far = False

        # --- Left detection ---
        for j in range(x):
            i = j + 2* x
            self.L_dist = msg.ranges[i]
            if msg.range_min < self.L_dist < self.desired_distance:
                self.lwall = True
                break

            # --- Left far detection ---
            if msg.ranges[i] > self.far:
                self.lwall_far = True
                break

        # --- Right detection ---
        for j in range(x):
            i = j + 7 * x
            self.R_dist = msg.ranges[i]
            if msg.range_min < self.R_dist < self.desired_distance:
                self.rwall = True
                break

            # --- Right far detection ---
            if msg.ranges[i] > self.far:
                self.rwall_far = True
        

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

        #self.get_logger().info(f"Detected - L:{self.lwall} F:{self.fwall} R:{self.rwall}")
        self.navigate(msg)

    def navigate(self, msg):
        #Reset movement flags
        self.move_forward_R = False
        self.move_forward_L = False
        self.move_forward = False
        self.turn_left = False
        self.turn_right = False

        # Wall-following state logic (LEFT wall hugger, clockwise)
        #self.get_logger().info(f"L:{self.lwall} F:{self.fwall} R:{self.rwall}")

        if self.direction == 'cw':
            if self.fwall:
                # Obstacle ahead → turn right
                self.turn_right = True
                self.get_logger().info('Front blocked → Turning RIGHT')
            elif self.lwall_far:
                # Check if wall is FAR on left → no walls slight turn right
                self.move_forward_R = True
                self.get_logger().info('Wall far → Finding wall')
            elif not self.lwall:
                # No left wall → turn left to reacquire
                self.turn_left = True
                self.get_logger().info('Left wall lost → Turning LEFT')
            else:
                # Wall on left → move forward
                self.move_forward = True
                self.get_logger().info('Following wall → Wall is' + str(self.L_dist) + 'm away')
        
        elif self.direction == 'ccw':
            
            if self.fwall:
                # Obstacle ahead → turn left
                self.turn_left = True
                self.get_logger().info('Front blocked → Turning LEFT')
            elif self.rwall_far:
                # Wall far on right → no walsl slight turn left
                self.move_forward_L = True
                self.get_logger().info('Wall far → Finding wall')
            elif not self.rwall:
                # No right wall → turn right to reacquire
                self.turn_right = True
                self.get_logger().info('Right wall lost → Turning RIGHT')
            else:
                # Wall on right → move forward
                self.move_forward = True
                self.get_logger().info('Following wall → Wall is' + str(self.R_dist) + 'm away')
    
    def set_velocity(self):
        msg = Twist()

        if self.move_forward_R:
            msg.linear.x = 0.25
            msg.angular.z = -0.01  # gentle right turn
        elif self.move_forward_L:
            msg.linear.x = 0.25
            msg.angular.z = 0.01
        elif self.move_forward:
            msg.linear.x = 0.25
            msg.angular.z = 0.0
        elif self.turn_left:
            msg.linear.x = 0.075
            msg.angular.z = 0.2
        elif self.turn_right:
            msg.linear.x = 0.075
            msg.angular.z = -0.2  # sharper right turn
    
        self.velocity_.publish(msg)
        #self.get_logger().info(f"Published cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}")  # DEBUG


def main(args=None):
    rclpy.init(args=args)
    node = Movement()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
