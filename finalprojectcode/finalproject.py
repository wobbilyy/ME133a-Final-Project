'''
final project
'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from finalprojectcode.GeneratorNode      import *
from finalprojectcode.TransformHelpers   import *
from finalprojectcode.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from finalprojectcode.KinematicChain     import KinematicChain

# Import the format for the condition number message
from std_msgs.msg import Float64

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Define the various points.

        # self.q0 = np.radians(np.array([0, 90, -90, 0, 0, 0]).reshape((-1,1)))
        self.q0 = np.array([0.0])
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1,1))
        self.R0 = Reye()

        # self.low_right  = np.array([-0.3, 0.5, 0.15]).reshape((-1,1))
        # self.low_left  = np.array([0.3, 0.5, 0.15]).reshape((-1,1))
        # self.phigh = np.array([0.0, 0.5, 0.9]).reshape((-1,1))

        # # Initialize the current/starting joint position.
        # self._lambda = 20
        # self.q  = self.q0

        # # Setup up the condition number publisher
        # self.pub = node.create_publisher(Float64, '/condition', 10)


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        # return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6']
        return ['theta1', 'theta2', 'theta3']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        return None
    
        # q = self.q0
        # qdot = np.zeros((3, 1))

        # # Return the position and velocity as python lists.
        # return (q.flatten().tolist(), qdot.flatten().tolist())


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
