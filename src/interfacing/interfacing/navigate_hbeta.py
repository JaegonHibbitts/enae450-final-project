# More Complex Maze Programs
#
# Perfect Heuristic Far execution
# Fastest approach
# Works well for SIMPLE disjoint mazes
# Where it's not great (PERFORMANCE DEPENDS ON HOW EXPLORATION IS UPDATED)
#  ______________________________
#  | S                          |
#  |____________   _____________|
#              |  END >>
#              |______ 
# Robot would bounce back and forth a limited number of times
# Between the ends of the wall
# Where it's excellent
# ___________________________________
# |                                 |
# |______________________________   |
#  << END                           |
# __________________________________|
# This algorithm will not backtrack
# Unless confronted with a distracting
# event such as a more appealing unknown
# 
# Bennefit: Fastest heuristic approach 
#           for simple mazes
# Note:     Designed for square mazes

from time import sleep

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np 

class RobotController(Node):
    def __init__(self):
        super().__init__("hbeta_robot_controller")

        # Sub to sensor
        self.Lidardis = self.create_subscription(LaserScan, "scan", self.obstacle, 10)

        # Controllers
        self.clock_speed = 0.5
        self.velocity_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(self.clock_speed, self.set_velocity) # rate of cmds
        
        # Calibration settings
        self.act_dist = 0.3
        self.exploration_timer_max = int(5 / self.clock_speed) # Units seconds to clocks
        self.old_heading = np.array([0, 0])
        self.dir_bias = 0.4 # NOTE: if set to zero will never loop back on self for exploration

        self.live_drot = 0

        # When you add a point to explored positons we bias against moving towards it
        self.explored_positions = [np.array([0, 0])]
        self.curr_pos = np.array([0, 0])
        self.exploration_weight = 0.4

        # Toggles
        self.exploration_timer = 1
        self.stop = 0
        self.lpub = False
        self.rpub = False

    # register far away points and store them
    # Heuristic way of tracking unknowns
    def navigate(self, msg):
        def pop_fill(i, i_arr, dist_arr, f=lambda x:x):
            nonlocal msg
            i = 0
            while len(dist_arr) < 8:
                if (msg.ranges[i] < msg.range_max):
                    dist_arr.append(msg.ranges[i])
                    i_arr.append(i)

                i += 1
                i %= len(msg.ranges)

            return False

        # Sensor collections
        noise_filter_i = []
        noise_filter_dist = []

        contender_filter_i = []
        contender_filter_dist = []

        # Rangemax
        i = 0
        pop_fill(i, noise_filter_i, noise_filter_dist)

        # Get the furthest distance
        dx = len(msg.ranges)//7
        for j in range(len(msg.ranges)):
            i = j + 0*dx # NOTE: For on the fly tuning
            for k, d in zip(noise_filter_i, noise_filter_dist):
                if (msg.ranges[i] > d) and (msg.ranges[i] < msg.range_max):
                    overflowed = pop_fill(msg, i, contender_filter_i, contender_filter_dist)
                    if (sum(noise_filter_dist)/len(noise_filter_dist) < sum(contender_filter_dist)/len(contender_filter_dist)):
                        noise_filter_dist = contender_filter_dist
                        noise_filter_i = contender_filter_i
                    contender_filter_dist = []
                    contender_filter_i = []

        absolute_sensor_position = sum(noise_filter_i) / len(noise_filter_i)
        
        # Reset noise filter
        noise_filter_i = []
        noise_filter_dist = []

        def vad(ang, dist): return np.array([dist*math.cos(ang), dist*math.sin(ang)])
        def adjust(v, delta_ang): 
            rot_mat = [[math.cos(delta_ang), -math.sin(delta_ang)], [-math.sin(delta_ang), math.cos(delta_ang)]]
            return np.matmul(rot_mat, v)
        
        # Check Exploration Graph
        # H0 Done with a temporary actuation graph
        _old_heading = self.old_heading
        drotation = self.live_drot - self.old_rotation
        old_heading = adjust(_old_heading, drotation) # Correct with rot mat ...
        
        # Get the furthest non negative heading distance (Reverse loss)
        # NOTE there exists an even more analog way of doing this
        # self._f(v) = ||(self.dir_bias - (heading dot norm(v))) * v|| # HEADING BIAS
        dexplored = [self.curr_pos - explored for explored in self.explored_pos] # NOTE all in lidar units
        # self._g(v) = avg over explored_pos || {v + (curr_pos} - self.explored_pos[i])|| # EXPLORATION BIAS
        # self._h(v) = self._f(v) + self._g(v) * self.exploration_weight
        # What this does self._h(v)
        _f = lambda v: np.linalg.norm((self.dir_bias - np.dot(old_heading, v / np.linalg.norm(v))) * v)
        _g = lambda v: numpy.average(np.linalg.norm(dexplored + np.full(shape=len(dexplored), fill_value=v)))
        _h = lambda v: _f(v) + _g(v) * self.exploration_weight
        
        # Fill the noise_filter
        i = 0
        pop_fill(i, noise_filter_i, noise_filter_dist, f=_h)

        # Increments
        da = 2. * math.pi / len(msg.ranges)
        for i in range(len(msg.ranges)):
            for k, d in zip(noise_filter_i, noise_filter_dist):
                if (msg.ranges[i] > d) and (msg.ranges[i] < msg.range_max):
                    overflowed = self.pop_fill(msg, i, contender_filter_i, contender_filter_dist, f=_h)
                    if (sum(noise_filter_dist)/len(noise_filter_dist) < sum(contender_filter_dist)/len(contender_filter_dist)):
                        noise_filter_dist = _f(vad(i*da, contender_filter_dist))
                        noise_filter_i = contender_filter_i
                    contender_filter_dist = []
                    contender_filter_i = []

        exploration_sensor_position = sum(noise_filter_i) / len(noise_filter_i)
        
        if math.abs(absolute_sensor_position - exploration_sensor_position) > 10:
            # Start exploration countdown (heading reset countdown)
            self.exploration_timer = self.exploration_timer_max

        # Actuation
        cutoff = len(msg.ranges)//3
        if sensor_position - cutoff < 0:        self.publ = True
        elif sensor_position - 2*cutoff > 0:    self.pubr = True
        
        self.act_dist = sum(noise_filter_dist) / len(noise_filter_dist)

    # Basic Obstacle Avoidance allows for more heuristic control
    def obstacle_avoidance(self, msg):
        """ Turn away from obstacles in Collision Box """
        self.get_logger().info('Avoiding Obstacles')
        dx = len(msg.ranges) // 8

        lwall = False
        rwall = False

        # Left
        for j in range(int(dx*1.1)): # NOTE: this sensor is slightly larger!
            i = j + 2*dx
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                lwall = True
                break

        # Right
        for j in range(dx):
            i = j + 5*dx
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                rwall = True
                break
        
        # Engage motor actions 
        if self.stop == 1: # Stopped
            if rwall: self.lpub = True
            elif lwall: self.rpub = True

    def obstacle(self, msg):
        x = len(msg.ranges)//8
        for j in range(x):
            i = j + 3*x
            if (msg.ranges[i] > self.act_dist * 0.9) and (msg.ranges[i] < msg.range_max):
                self.stop = 1
                self.obstacle_avoidance(msg)
                self.wind_down = 10
            else:
                self.publ = False
                self.pubr = False
                self.stop = 0

                if self.wind_down > 0: self.wind_down -= 1
                else:
                    # NOTE can limit the amount of times called for performance
                    self.navigate(msg)
    
    def dec_exploration_timer(self):
        """
        Only a certain amount of time is allotted to moving away
        from heading
        """
        self.exploration_timer -= 1
    
    def set_velocity(self):
        def vad(ang, dist): return np.array([dist*math.cos(ang), dist*math.sin(ang)])
        
        # Update the current position
        self.curr_pos += self.vad(self.live_dheading, self.live_drot)

        # Move the Robot
        msg = Twist()
        if (self.stop == 0):
           msg.linear.x = .1
           self.get_logger().info('Moving')

           # velocity 0.1 for 1 second to lidar coords = 0.098 est 
           # TODO Calibrate
           self.heading = 0.098 * self.clock_speed
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
        
        # To start exploration_timer is set to exploration_timer_max
        if self.exploration_timer > 0:
            self.dec_exploration_timer()
        elif self.exploration_timer == 0:
            self.exploration_timer -= 1 # Only reset once (Assumed by now you've nearly found the next direction)
            # Reset heading to new heading
            self.live_dheading = np.array([0, 0])
            self.old_rotation = self.live_drot
            self.old_heading = vad(self.old_rotation, self.live_dheading) # NOTE: Storing the old rotation
            self.explored_positions.append(self.curr_pos) # One place to add it
        
        # TODO: Can also add explored_points at a constant rate.

    def turnL(self):
        msg = Twist()
        msg.angular.z = -.3 # Good, speed of rotate away from opposing obstacle
        self.velocity_.publish(msg)

        # Called at clock interval (TODO CALIBRATE)
        self.live_drot += -math.pi/12 * self.clock_speed

    def turnR(self):
        msg = Twist()
        msg.angular.z = .3 # Good, speed of rotate away from opposing obstacle
        self.velocity_.publish(msg)

        # Called at clock interval (TODO CALIBRATE)
        self.live_drot += math.pi/12 * self.clock_speed


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
