from time import sleep

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class movement(Node):
    def __init__(self):
        super().__init__("number_publisher")
        #create node class
        self.i = 0
        self.forward_velocity_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.set_velocity)
        
        msg = Twist()
        msg.linear.x = float(.0)

        self.forward_velocity_.publish(msg)

        # create publisher under type Twist
       
    def set_velocity(self):
        msg = Twist()
        if self.i < 6:
           msg.linear.x = float(.1)
        else:
           msg.linear.x = float(.0)

        # set linear velocity
        self.forward_velocity_.publish(msg)
        #publish message
        self.get_logger().info(f'Publishing: "{msg.linear.x}"')
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
