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
        self.timer = self.create_timer(0.5, self.set_velocity)
        # create publisher under type Twist
        self.i = 0
        self.stop = 0
       
    def obstacle(self,msg):
        x = len(msg.ranges)//3
        for j in range(x):
            i = j + x
            if (msg.ranges[i] < .5) and (msg.ranges[i] > msg.range_min):
                self.stop = 1
                break
            else:
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
        self.get_logger().info('Publishing: "%d"' % msg.linear.x)
        self.i += 1
        #log publish

def main(args=None):
    rclpy.init(args=args)
    node = movement()
    # make node
    rclpy.spin(node)
    rclpy.shutdown()
    # shutdown node


if __name__ == '__main__':
    main()
