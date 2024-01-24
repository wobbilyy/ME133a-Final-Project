'''
This file demos the inverse kinematics of the platform by making it follow
a defined trajectory.

When executed, the platform will move side to side, up and down, and tilt.
'''

import rclpy
import numpy as np

# Grab the utilities
from finalprojectcode.GeneratorNode      import GeneratorNode
from finalprojectcode.TransformHelpers   import *
from finalprojectcode.TrajectoryUtils    import *

from finalprojectcode.KinematicChain   import *

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        r =  1.25
        height = 2.6

        self.BASE_MOUNTS_POS = [[ 0.25     ,  1.5       , 0],
                        [-0.25     ,  1.5       , 0],
                        [-1.424038 , -0.533494  , 0],
                        [-1.1174038, -0.96650635, 0], 
                        [ 1.1174038, -0.96650635, 0], 
                        [ 1.424038 , -0.533494  , 0]]
        self.PLATFORM_CENTER = [0,  0, height]
        self.PLATFORM_MOUNTS_POS = [[r * np.cos(np.radians(60)) , r * np.sin(np.radians(60)) , height ],
                        [r * np.cos(np.radians(120)), r * np.sin(np.radians(120)), height ],
                        [r * np.cos(np.radians(180)), r * np.sin(np.radians(180)), height ],
                        [r * np.cos(np.radians(240)), r * np.sin(np.radians(240)), height ],
                        [r * np.cos(np.radians(300)), r * np.sin(np.radians(300)), height ],
                        [r * np.cos(np.radians(0))  , r * np.sin(np.radians(0))  , height ]] 

        self.KinematicChain = KinematicChain(self.PLATFORM_MOUNTS_POS, self.PLATFORM_CENTER, self.BASE_MOUNTS_POS)

        # # Define the various points.
        self.q0 = [0, 0, 0+height, np.radians(0), np.radians(0), np.radians(0)]
        self.sq0 = self.KinematicChain.stewart_to_spider_q(self.q0)

        self.q_up = [0, 0, 1+height, np.radians(0), np.radians(0), np.radians(0)]
        self.sq_up = self.KinematicChain.stewart_to_spider_q(self.q_up)

        self.q_right = [1, 0, 1+height, np.radians(15), np.radians(0), np.radians(0)]
        # self.x_right = [1, 0, 0, np.radians(0), np.radians(0), np.radians(0)]
        self.sq_right = self.KinematicChain.stewart_to_spider_q(self.q_right)

        self.q_left = [-1, 0, 1+height, np.radians(15), np.radians(25), np.radians(0)]
        # self.x_left = [-1, 0, 0, np.radians(0), np.radians(0), np.radians(0)]
        self.sq_left = self.KinematicChain.stewart_to_spider_q(self.q_left)

        self.q_back = [0, 1, 1+height, np.radians(-20), np.radians(0), np.radians(0)]
        self.sq_back = self.KinematicChain.stewart_to_spider_q(self.q_back)

        self.q_forward = [0, -1, 0+height, np.radians(0), np.radians(-10), np.radians(0)]
        self.sq_forward = self.KinematicChain.stewart_to_spider_q(self.q_forward)

        self.demo = [
            np.array(self.sq0),
            np.array(self.sq_up),
            np.array(self.sq_right),
            np.array(self.sq_left),
            np.array(self.sq_back),
            np.array(self.sq_forward),
        ]
        self.demo_positions = [
            np.array(self.q0),
            np.array(self.q_up),
            np.array(self.q_right),
            np.array(self.q_left),
            np.array(self.q_back),
            np.array(self.q_forward),
        ]
        self.left_right = [
            np.array(self.sq0),
            np.array(self.sq_up),
            # np.array(self.q_left),
            # np.array(self.q_right)
        ]
        self.left_right_positions = [
            np.array(self.q0),
            np.array(self.q_up)
            # np.array(self.x_left),
            # np.array(self.x_right)
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
        #### FULL DEMO ####
        if t > self.desired_time * len(self.demo):
            return None

        if self.last_time == -1:
            self.last_time = t

        delta = t - self.last_time

        if delta > self.desired_time:
            self.last_time = t
            self.index = (self.index + 1) % len(self.demo)
            delta = 0

        next_index = (self.index + 1) % len(self.demo)

        sqdot = ((self.demo[next_index] - self.demo[self.index]) / self.desired_time)
        sq = self.demo[self.index] + sqdot * delta

        qdot = ((self.demo_positions[next_index] - self.demo_positions[self.index]) / self.desired_time)
        q = self.demo_positions[self.index] + qdot * delta
        q = q.flatten().tolist()

        # Return the position and velocity as python lists.
        return (sq.flatten().tolist(), sqdot.flatten().tolist(), q)

#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, unp.sing the above
    # Trajectory class.
    T = Trajectory
    generator = GeneratorNode('generator', 100, T)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message pasnp.sing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
