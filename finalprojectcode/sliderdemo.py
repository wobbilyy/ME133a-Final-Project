'''
final project
'''

import rclpy
import numpy as np
from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

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

        self.base_pos = [[ 0.25     ,  1.5       , 0],
                         [-0.25     ,  1.5       , 0],
                         [-1.424038 , -0.533494  , 0],
                         [-1.1174038, -0.96650635, 0], 
                         [ 1.1174038, -0.96650635, 0], 
                         [ 1.424038 , -0.533494  , 0]]
        self.center_pos = [0,  0, height - 0.2]
        self.top_pos = [[r * np.cos(np.radians(60)) , r * np.sin(np.radians(60)) , height - 0.2],
                        [r * np.cos(np.radians(120)), r * np.sin(np.radians(120)), height - 0.2],
                        [r * np.cos(np.radians(180)), r * np.sin(np.radians(180)), height - 0.2],
                        [r * np.cos(np.radians(240)), r * np.sin(np.radians(240)), height - 0.2],
                        [r * np.cos(np.radians(300)), r * np.sin(np.radians(300)), height - 0.2],
                        [r * np.cos(np.radians(0))  , r * np.sin(np.radians(0))  , height - 0.2]]

        self.KinematicChain = KinematicChain(self.top_pos, self.center_pos, self.base_pos)
        
        # # Define the various points.
        self.x0 = [0, 0, 0, 0, 0, 0]                                     # initial x
        self.q0 = self.KinematicChain.stewart_to_spider_q(self.x0)       # initial q spider


        # Define the various points.
        self.q = self.q0                    # q spider 
        self.x = np.array(self.x0) + 2.4    # x ([Tx, Ty, Tz, psi, theta, phi])


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
        qdot = np.zeros((18, 1))
        q = np.array(self.KinematicChain.stewart_to_spider_q(np.array(self.x) - 2.4))

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist(), np.array(self.x) - 2.4)

#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, unp.sing the above
    # Trajectory class.
    T = Trajectory
    generator = GeneratorNode('generator', 100, T, True)
    
    generator.run_gui()
    
    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
