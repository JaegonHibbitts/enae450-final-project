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
        # subscribe to sensor
       
        self.forward_velocity_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.set_velocity)
        # create publisher under type Twist
        self.i = 0
        self.stop = 0

    def navigate(self,msg):
        if self.stop == 1:
            # when the robot is stopped
            x = len(msg.ranges)//6
            for j in range(x):
                i = j + x
                if (msg.ranges[i] > 1) and (msg.ranges[i] < msg.range_max):
                    while(self.stop == 1):
                        self.turnL()
                    self.turnL()
                    break
                # check left range, if the range on the left is open, turn left
            for j in range(x):
                i = j + 4*x
                if (msg.ranges[i] > 1) and (msg.ranges[i] < msg.range_max):
                    while(self.stop == 1):
                        self.turnR()
                    self.turnR()
                    break
                # check right range, if the range on the right is open, turn right
            while(self.stop == 1):
                self.turnL()
            self.turnL()
            # if there are obstacles everywhere, turn left


       
    def obstacle(self,msg):
        pub = Twist()
        x = len(msg.ranges)//8
        # for the range infront of the robot, check lidar
        for j in range(2*x):
            i = j + 3*x
            if (msg.ranges[i] < .5) and (msg.ranges[i] > msg.range_min):
                self.stop = 1
                self.get_logger().info('Stopped')
                break
                # if there is an obstacle stop robot
            else:
                self.stop = 0
                self.get_logger().info('Moving')
                # else let robot move forward.

        if self.stop == 0:
            # if the robot is moving
            if max(msg.ranges[3*x : 4*x]) > max(msg.ranges[4*x : 5*x]):
                pub.angular.z = .1
                # if the largest lidar value is to the left, go left
            else:
                pub.angular.z = -.1
                # if the largest lidar value is to the right, go right
            self.forward_velocity_.publish(pub)
            self.get_logger().info('Publishing Angular V nav: "%f"' % pub.angular.z)

        self.navigate(msg)
        #avoid obstacle
      
                
    def set_velocity(self):
        msg = Twist()
        if (self.stop == 0):
           msg.linear.x = .1
           # if robot is not stopped, set linear velocity to .1
        else:
           msg.linear.x = .0
        # set linear velocity
        self.forward_velocity_.publish(msg)
        #publish message
        self.get_logger().info('Publishing Linear V: "%f"' % msg.linear.x)
        self.i += 1
        #log publish

    def turnL(self):
        msg = Twist()
        if self.stop == 1:
            msg.angular.z = .1
        else:
            msg.angular.z = 0
            self.forward_velocity_.publish(msg)
        self.get_logger().info('Publishing Angular V Obst: "%f"' % msg.angular.z)

    def turnR(self):
        msg = Twist()
        if self.stop == 1:
            msg.angular.z = -.1
        else:
            msg.angular.z = 0
        self.forward_velocity_.publish(msg)
        self.get_logger().info('Publishing Angular V Obst: "%f"' % msg.angular.z)


def main(args=None):
    rclpy.init(args=args)
    node = movement()
    # make node
    rclpy.spin(node)
    rclpy.shutdown()
    # shutdown node


if __name__ == '__main__':
    main()
