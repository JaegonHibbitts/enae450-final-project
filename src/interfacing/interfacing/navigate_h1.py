# VERY weak algorihtm
# CAN be tuned to the maze
#
# Hyperparameters
#  1. Commitment Clocks

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import collections

class CollisionBox(collections.namedtuple('CBox', ['hw', 'l']):
    """ Collision Box
    Represents a virtual collision box at the front of the robot.
    Has a half-width and a length
    """
    __slots__ = ()

class Movement(Node):
    def __init__(self):
        super().__init__("number_publisher")
       
        self.Lidardis = self.create_subscription(LaserScan, "scan", self.obstacle, 10)
       
        self.seconds_per_clock = 0.25
        self.velocity_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(self.seconds_per_clock, self.set_velocity)
        
        # Settings
        self.stop = 0
        self.act_dist = 0.6

        self.cbox = CollisionBox(hw=0.3, l=0.4)

        # Noise Tolerance Controls
        self.sensor_size = 8

        # Toggle Controllers
        self.toggle = False
        self.lpub = False
        self.rpub = False

        self.blpub = False # contains both lr

        self.i = 0
        
        # Timed Controllers
        self.commit_clocks = 0
        self.commit_clocks_max = 100 # clocks

        self.turn_timer_slow = 0
        self.turn_timer_slow_max = 1 # seconds

    def pop_fill(self, msg, i, i_arr, dist_arr):
        while len(dist_arr) < self.sensor_size:
            if (msg.ranges[i] < msg.range_max):
                dist_arr.append(msg.ranges[i])
                i_arr.append(i)

            i += 1 # CAN ERR ...
            i %= len(msg.ranges)
            # Can buggie if dist too large! ...

        return False

    def sense(self, msg):
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
   
        sensor_position = sum(noise_filter_i)/len(noise_filter_i)
        sensor_dist = sum(noise_filter_dist)/len(noise_filter_dist) 

        return (sensor_dist, sensor_position)   


    def navigate(self,msg):
        (sensor_dist, sensor_position) = sense(self, msg)

        bias_cutoff = len(msg.ranges)//2

        #angle = ((7 - len(msg.ranges)//2) / len(msg.ranges)) * 2 * math.pi / 2 # Compiler opt
        angle = ((sensor_position / len(msg.ranges)) + 0.5) * math.pi
        ydist = math.sin(angle) * sensor_dist
        axdist = math.abs(math.cos(angle) * sensor_dist)

        # Toggle Controllers
        collider_hit = axdist > self.cbox.hw and ydist < self.cbox.l and ydist > 0
        if collider_hit: self.stop = 1 # Puts into default state
        elif sensor_position - bias_cutoff > 0: self.rpub = True
        elif sensor_position - bias_cutoff < 0: self.lpub = True

        # To unstuck allow default move away from sensor detection
        # NOTE: Could make Analog
        self.blpub = sensor_position - bias_cutoff > 0


    def obstacle(self,msg):
        x = len(msg.ranges)//7

        maybe_open_ahead = True
        for j in range(x):
            i = j + 3*x
            # If one point in the range is worse than the act_dist 
            # (Then we're committing)
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] < msg.range_max):
                maybe_open_ahead = False
                break
        
        if not maybe_open_ahead:
            # Look for a better distance
            self.stop = 1
            self.commit_clocks = 0 # Reset commitment clock

            self.navigate(msg)
        else:
            # Commit
            self.lpub = False
            self.rpub = False
            self.stop = 0

            # Commitment Clock
            self.commit_clocks = (self.clock + 1) % self.commit_clocks_max
            if self.commit_clocks == 0:
                self.navigate(msg)
    
    def set_velocity(self):
        msg = Twist()
        if self.stop == 0:
           msg.linear.x = .5
           self.get_logger().info('Moving')
        else:
           msg.linear.x = .0
           self.get_logger().info('Stopping')

        # set linear velocity
        self.velocity_.publish(msg)
        
        
        #if self.turn_timer_slow % self.turn_timer_slow_max == 0:
        if self.lpub:
            self.turnL()
            self.lpub = False
            self.get_logger().info('Turning L')
        elif self.rpub:
            self.turnR()
            self.rpub = False
            self.get_logger().info('Turning R')
        elif self.stop:
            self.slow_release()
            self.get_logger().info('Releasing')
        
        self.turn_timer_slow += self.seconds_per_clock
        self.turn_timer_slow %= self.turn_timer_slow_max
    

    def slow_release(self):
        # Make sure robot does not get stuck
        msg = Twist()
        msg.linear.x = .2
        msg.angular.z = .2 * (-1 if self.blpub else 1)
        self.velocity_.publish(msg)

    def turnL(self):
        msg = Twist()
        msg.angular.z = -.4
        self.velocity_.publish(msg)

    def turnR(self):
        msg = Twist()
        msg.angular.z = .4
        self.velocity_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    node = Movement()

    # Run the node
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
