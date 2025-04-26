# This implementation moves around a wall maze.
# If there is no block to either side the robot
# wraps in a calibrated around the last block it saw.
# 
# Visual
#     [?]
# [][] } Calibrated arc
# [] r-
# [] |

from time import sleep

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy

class movement(Node):
    def __init__(self):
        super().__init__("number_publisher")

        # Sub to sensor
        self.Lidardis = self.create_subscription(LaserScan, "scan", self.obstacle, 10)

        # Controllers
        self.velocity_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.set_velocity) # rate of cmds
        
        # Calibration settings
        self.act_dist = 0.3

        # Toggles
        self.stop = 0
        self.lpub = False
        self.rpub = False

    def navigate(self,msg):
        self.get_logger().info('Turning')
        x = len(msg.ranges) // 8

        lwall = False
        rwall = False
        # Left
        for j in range(int(x*1.1)): # NOTE: this sensor is slightly larger!
            i = j + 2*x
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                lwall = True

        # Right
        for j in range(x):
            i = j + 5*x
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                rwall = True
        
        # Engage motor actions 
        if self.stop == 1: # Stopped
            if rwall: self.lpub = True
            elif lwall: self.rpub = True
        elif not rwall and not lwall: 
            self.lpub = True
        else:
            # Stop rotation here (No timer callback)
            msg = Twist()
            msg.angular.z = .0
            self.velocity_.publish(msg)


    def obstacle(self,msg):
        x = len(msg.ranges)//8
        for j in range(2*x):
            i = j + 3*x
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                self.stop = 1
                break
            else:
                self.stop = 0
    
        self.navigate(msg)


    def set_velocity(self):
        msg = Twist()
        if (self.stop == 0):
           msg.linear.x = .1
           self.get_logger().info('Moving')
        else:
           msg.linear.x = .0
           self.get_logger().info('Stopping')

        # set linear velocity
        self.velocity_.publish(msg)

        # TODO test and calibrate
        if self.lpub:
            self.turnL()
            self.lpub = False
            self.get_logger().info('Turning L')
        elif self.rpub:
            self.turnR()
            self.rpub = False
            self.get_logger().info('Turning R')
        

    def turnL(self):
        msg = Twist()
        msg.angular.z = -.3 # Calibrate the arc
        self.velocity_.publish(msg)

    def turnR(self):
        msg = Twist()
        msg.angular.z = .3 # Good, speed of rotate away from opposing obstacle
        self.velocity_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = movement()
    # make node
    rclpy.spin(node)
    rclpy.shutdown()
    # shutdown node


if __name__ == '__main__':
    main()
