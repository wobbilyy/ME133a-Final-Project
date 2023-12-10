'''
final project
'''

import rclpy
import numpy as np
from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from finalprojectcode.GeneratorNode      import GeneratorNode
from hw5code.TransformHelpers   import *
from hw5code.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from finalprojectcode.KinematicChain     import KinematicChain

from finalprojectcode.KinematicHelpers   import *

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # # Set up the kinematic chain object.
        r =  1.25
        height = 2.6
        base_pos = [[ 0.25     ,  1.5       , 0],
                    [-0.25     ,  1.5       , 0],
                    [-1.424038 , -0.533494  , 0], 
                    [-1.1174038, -0.96650635, 0], 
                    [ 1.1174038, -0.96650635, 0], 
                    [ 1.424038 , -0.533494  , 0]]
        center_pos = [0        ,  0         , height - 0.2]
        top_pos = [[r * np.cos(np.radians(0))  , r * np.sin(np.radians(0))  , height - 0.2],
                   [r * np.cos(np.radians(60)) , r * np.sin(np.radians(60)) , height - 0.2],
                   [r * np.cos(np.radians(120)), r * np.sin(np.radians(120)), height - 0.2],
                   [r * np.cos(np.radians(180)), r * np.sin(np.radians(180)), height - 0.2],
                   [r * np.cos(np.radians(240)), r * np.sin(np.radians(240)), height - 0.2],
                   [r * np.cos(np.radians(300)), r * np.sin(np.radians(300)), height - 0.2]] 

        self.KinematicChain = KinematicHelpers(top_pos, center_pos, base_pos)

        # # Define the various points.
        self.x0 = [0, 0, 0, np.radians(0), np.radians(0), np.radians(0)]
        self.q0 = self.KinematicChain.stewart_to_spider_q(self.x0)

        self.x_up = [0, 0, 1, np.radians(0), np.radians(0), np.radians(0)]
        self.q_up = self.KinematicChain.stewart_to_spider_q(self.x_up)

        self.x_right = [1, 0, 1, np.radians(15), np.radians(0), np.radians(0)]
        self.q_right = self.KinematicChain.stewart_to_spider_q(self.x_right)

        self.x_left = [-1, 0, 1, np.radians(15), np.radians(25), np.radians(0)]
        self.q_left = self.KinematicChain.stewart_to_spider_q(self.x_left)

        self.x_back = [0, 1, 1, np.radians(-20), np.radians(0), np.radians(0)]
        self.q_back = self.KinematicChain.stewart_to_spider_q(self.x_back)

        self.x_forward = [0, -1, 0, np.radians(0), np.radians(-10), np.radians(0)]
        self.q_forward = self.KinematicChain.stewart_to_spider_q(self.x_forward)

        self.demo = [
            np.array(self.q0),
            np.array(self.q_up),
            np.array(self.q_right),
            np.array(self.q_left),
            np.array(self.q_back),
            np.array(self.q_forward),
        ]
        self.left_right = [
            np.array(self.q0),
            np.array(self.q_up),
            np.array(self.KinematicChain.stewart_to_spider_q([1, 0, 1, np.radians(0), np.radians(0), np.radians(0)])),
            np.array(self.KinematicChain.stewart_to_spider_q([-1, 0, 0.5, np.radians(0), np.radians(0), np.radians(0)]))
        ]
        self.last_time = -1
        self.desired_time = 2
        self.index = 0

        # Initialize the current/starting joint position.
        self._lambda = 20

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['leg1-pitch', 'leg1-roll', 'leg1-prismatic',
                'leg2-pitch', 'leg2-roll', 'leg2-prismatic',
                'leg3-pitch', 'leg3-roll', 'leg3-prismatic',
                'leg4-pitch', 'leg4-roll', 'leg4-prismatic',
                'leg5-pitch', 'leg5-roll', 'leg5-prismatic',
                'leg6-pitch', 'leg6-roll', 'leg6-prismatic']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        if t > self.desired_time * 2 * len(self.left_right):
            return None

        if self.last_time == -1:
            self.last_time = t

        delta = t - self.last_time

        if delta > self.desired_time:
            self.last_time = t
            self.index = (self.index + 1) % len(self.left_right)
            delta = 0

        next_index = (self.index + 1) % len(self.left_right)

        qdot = ((self.left_right[next_index] - self.left_right[self.index]) / self.desired_time)
        q = self.left_right[self.index] + qdot * delta

        # if t > self.desired_time * len(self.demo):
        #     return None

        # if self.last_time == -1:
        #     self.last_time = t

        # delta = t - self.last_time

        # if delta > self.desired_time:
        #     self.last_time = t
        #     self.index = (self.index + 1) % len(self.demo)
        #     delta = 0

        # next_index = (self.index + 1) % len(self.demo)

        # qdot = ((self.demo[next_index] - self.demo[self.index]) / self.desired_time)
        # q = self.demo[self.index] + qdot * delta

        # qdot = np.zeros((18, 1))
        # q = np.array(self.q_left)

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
