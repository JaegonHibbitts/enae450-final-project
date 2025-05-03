# VERY weak algorihtm

from time import sleep

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy

class movement(Node):
    def __init__(self):
        super().__init__("number_publisher")
        #create node class
       
        self.Lidardis = self.create_subscription(LaserScan, "scan", self.obstacle, 10)
       
        self.forward_velocity_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(0.3, self.set_velocity)
        # create publisher under type Twist
        self.i = 0
        self.stop = 0

        self.act_dist = 0.3

        self.toggle = False
        self.lpub = False
        self.rpub = False

    def pop_fill(self, msg, i, i_arr, dist_arr):
        i = 0
        while len(dist_arr) < 8:
            if (msg.ranges[i] < msg.range_max):
                dist_arr.append(msg.ranges[i])
                i_arr.append(i)

            i += 1 # CAN ERR ...
            # Can buggie if dist too large! ...

        return False
    

    def navigate(self,msg):
        noise_filter_i = []
        noise_filter_dist = []

        contender_filter_i = []
        contender_filter_dist = []

        # Rangemax
        i = 0
        self.pop_fill(msg, i, noise_filter_i, noise_filter_dist)

        x = len(msg.ranges)//7
        for j in range(len(msg.ranges)):
            i = j
            for k, d in zip(noise_filter_i, noise_filter_dist):
                if (msg.ranges[i] > d) and (msg.ranges[i] < msg.range_max):
                    overflowed = self.pop_fill(msg, i, contender_filter_i, contender_filter_dist)
                    if (sum(noise_filter_dist)/len(noise_filter_dist) < sum(contender_filter_dist)/len(contender_filter_dist)):
                        noise_filter_dist = contender_filter_dist
                        noise_filter_i = contender_filter_i
                    contender_filter_dist = []
                    contender_filter_i = []
   
        cutoff = len(msg.ranges)//3
        sensor_position = sum(noise_filter_i)/len(noise_filter_i)

        if sensor_position - cutoff < 0:        self.publ = True; self.pubr = False
        elif sensor_position - 2*cutoff > 0:    self.pubr = True; self.publ = False
        
        self.act_dist = sum(noise_filter_dist)/len(noise_filter_dist)


    def obstacle(self,msg):
        x = len(msg.ranges)//8
        for j in range(x):
            i = j + 3*x
            if (msg.ranges[i] > self.act_dist * 0.9) and (msg.ranges[i] < msg.range_max):
                self.stop = 1
                self.navigate(msg)
            else:
                self.publ = False
                self.pubr = False
                self.stop = 0
      
                
    def set_velocity(self):
        msg = Twist()
        if (self.stop == 0):
           msg.linear.x = .1
        else:
           msg.linear.x = .0
        
        # set linear velocity
        self.forward_velocity_.publish(msg)
        
        #publish message
        self.i += 1

        #log publish
        self.get_logger().info('Publishing: "%d"' % msg.linear.x)
        
        if self.lpub:
            self.turnL()
        elif self.rpub:
            self.turnR()

    def turnL(self):
        msg = Twist()
        if self.stop == 1:
            msg.angular.z = -.3
        else:
            msg.angular.z = .0
        
        self.get_logger().info("turnL" % msg.linear.x)
        self.forward_velocity_.publish(msg)

    def turnR(self):
        msg = Twist()
        if self.stop == 1:
            msg.angular.z = .3
        else:
            msg.angular.z = .0

        self.get_logger().info("turnR" % msg.linear.x)
        self.forward_velocity_.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = movement()
    # make node
    rclpy.spin(node)
    rclpy.shutdown()
    # shutdown node


if __name__ == '__main__':
    main()
