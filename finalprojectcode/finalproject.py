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

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.leg1_chain = KinematicChain(node, 'world', 'leg1-tip', self.jointnames()[:3])
        self.leg2_chain = KinematicChain(node, 'world', 'leg2-tip', self.jointnames()[3:6])
        self.leg3_chain = KinematicChain(node, 'world', 'leg3-tip', self.jointnames()[6:9])
        self.leg4_chain = KinematicChain(node, 'world', 'leg4-tip', self.jointnames()[9:12])
        self.leg5_chain = KinematicChain(node, 'world', 'leg5-tip', self.jointnames()[12:15])
        self.leg6_chain = KinematicChain(node, 'world', 'leg6-tip', self.jointnames()[15:])

        # Define the various points.
        self.q0 = np.radians(np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1,1))
        self.R0 = Reye()

        self.qgoal = np.array([0, 0, 1.0, 0, 0, 1.0, 0, 0, 1.0, 0, 0, 1.0, 0, 0, 1.0, 0, 0, 1.0]).reshape((-1,1))
        self.solutions = [
            np.array([0, 0, 1.0, 0, 0, 1.0, 0, 0, 1.0, 0, 0, 1.0, 0, 0, 1.0, 0, 0, 1.0]).reshape((-1,1)),
            np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape((-1,1))
        ]
        self.last_time = -1
        self.desired_time = 10
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
        if self.last_time == -1:
            self.last_time = t

        delta = t - self.last_time

        if delta > self.desired_time:
            self.last_time = t
            self.index = (self.index + 1) % 2
            delta = 0

        next_index = (self.index + 1) % 2

        qdot = ((self.solutions[next_index] - self.solutions[self.index]) / self.desired_time)
        q = self.solutions[self.index] + qdot * delta

        # qdot = np.zeros((18, 1))
        # q = self.q0

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
