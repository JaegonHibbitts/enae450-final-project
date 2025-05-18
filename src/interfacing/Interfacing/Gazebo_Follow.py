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
        self.seconds_per_clock = 0.25
        self.velocity_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(self.seconds_per_clock, self.set_velocity) # rate of cmds
        
        # Calibration settings =================!!
        self.act_dist = 0.5 #CHANGE THIS
        self.obs_dist = 0.4 # CHANGE THIS

        # Toggles
        self.stop = 0
        self.lpub = False
        self.rpub = False
        self.slpub = False
        self.srpub = False # Unused
        self.turn_timer_slow = 0
        self.turn_timer_slow_max = 1 # seconds

        self.turn_timer_fast = 0
        self.turn_timer_fast_max = 0.5 # seconds

    def navigate(self,msg):
        self.get_logger().info('Turning')
        x = len(msg.ranges) // 10

        lwall = False
        rwall = False
        rwall_close = False
        lwall_close = False
        
        # =============== SNESOR CALIBRATIONS =================
        #This is the range of the LIDAR point front left of robot
        for j in range(2*x): # NOTE: this sensor is slightly larger!
            i = j + 0*x
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                lwall = True
                self.get_logger().info('Left wall detected')

        #This is the range of the LIDAR point front right of robot
        for j in range(2*x):
            i = j + 8*x
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                rwall = True
                self.get_logger().info('Right wall detected')
        
        #This is the range of the LIDAR pointing straight left of robot
        for j in range(x):
            i = j + 2*x
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                lwall_close = True
                self.get_logger().info('Right wall detected')
        
        #This is the range of the LIDAR pointing straight right of robot
        for j in range(x):
            i = j + 7*x
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                rwall_close = True
                self.get_logger().info('Right wall detected')
        
        # Engage motor actions     
        if not rwall and not lwall: 
            self.slpub = True
            self.get_logger().info('No detection')
        elif lwall: self.rpub = True
        elif rwall: self.lpub = True
        elif lwall_close: self.rpub_fast = True
        elif rwall_close: self.lpub_fast = True
        
        else:
            # Stop rotation here (No timer callback)
            msg = Twist()
            msg.angular.z = .0
            self.velocity_.publish(msg)


    def obstacle(self,msg):
        x = len(msg.ranges)//8
        # Front
        for j in range(2*x):
            i = j + 0*x
            if (msg.ranges[i] < self.obs_dist) and (msg.ranges[i] > msg.range_min):
                self.stop = 1
                break
            else:
                self.stop = 0
    
        self.navigate(msg)


    #CHANGE ANGULAR X TO MAKE MOVE FOWARD FASTER
    def set_velocity(self):
        msg = Twist()
        if (self.stop == 0):
           msg.linear.x = .125
           self.get_logger().info('Moving')
        else:
           msg.linear.x = .0
           self.get_logger().info('Stopping')

        # set linear velocity
        self.velocity_.publish(msg)
        
        # TODO test and calibrate
        if self.turn_timer_fast % self.turn_timer_fast_max == 0:
            if self.lpub:
                self.turnL()
                self.lpub = False
                self.get_logger().info('Quick Turning L')
            elif self.rpub:
                self.turnR()
                self.rpub = False
                self.get_logger().info('Quick Turning R')
        
        if self.turn_timer_slow % self.turn_timer_slow_max == 0:
            if self.slpub:
                self.turnL()
                self.slpub = False
                self.get_logger().info('Turning L')
            elif self.srpub:
                self.turnR()
                self.srpub = False
                self.get_logger().info('Turning R')
        
        self.turn_timer_fast += self.seconds_per_clock
        self.turn_timer_slow += self.seconds_per_clock

        self.turn_timer_fast %= self.turn_timer_fast_max
        self.turn_timer_slow %= self.turn_timer_slow_max

    #CHAGE ANGULAR Z TO MAKE TURN FASTER
    def turnL(self):
        msg = Twist()
        msg.angular.z = 0.175 # Calibrate the arc
        self.velocity_.publish(msg)

    def turnR(self):
        msg = Twist()
        msg.angular.z = -0.175 # Good, speed of rotate away from opposing obstacle
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
