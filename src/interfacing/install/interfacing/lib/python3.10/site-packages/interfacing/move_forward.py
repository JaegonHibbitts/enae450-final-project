from time import sleep

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class movement(Node):
    def __init__(self):
        super().__init__("number_publisher")
        #create node class
       
        self.forward_velocity_ = self.create_publisher(Twist, "cmd_vel", 10)
        # create publisher under type Twist
       
    def set_velocity(self,value):
    msg = Twist()
    msg.linear.x = value
    # set linear velocity
    self.forward_velocity_.publish(msg)
    #publish message
    self.get_logger().info('Publishing: "%d"' % msg.data)
    #log publish

def main(args=None):
    rclpy.init(args=args)
    node = movement()
    # make node
    node.set_velocity(.1)
    # set velocity
    # node.set_velocity(0)
    # make node
    rclpy.spin(node)
    rclpy.shutdown()
    # shutdown node


if __name__ == '__main__':
    main()
