'''
This file demos the forward kinematics of the Stewart platform. 

When executed, 6 sliders controlling the leg lengths are displayed and the 
platform position and orientation is updated in real time.
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
        self.PLATFORM_CENTER = [0,  0, height ]
        self.PLATFORM_MOUNTS_POS = [[r * np.cos(np.radians(60)) , r * np.sin(np.radians(60)) , height ],
                        [r * np.cos(np.radians(120)), r * np.sin(np.radians(120)), height ],
                        [r * np.cos(np.radians(180)), r * np.sin(np.radians(180)), height ],
                        [r * np.cos(np.radians(240)), r * np.sin(np.radians(240)), height ],
                        [r * np.cos(np.radians(300)), r * np.sin(np.radians(300)), height ],
                        [r * np.cos(np.radians(0))  , r * np.sin(np.radians(0))  , height ]]

        self.KinematicChain = KinematicChain(self.PLATFORM_MOUNTS_POS, self.PLATFORM_CENTER, self.BASE_MOUNTS_POS)
        
        # # Define the various points.
        self.q0 = [0, 0, 0 + height, np.radians(0), np.radians(0), np.radians(0)] # initial q
        self.sq0 = self.KinematicChain.stewart_to_spider_q(self.q0)               # initial q spider
        self.x0 = [height, height, height, height, height, height]

        # Define the various points.
        self.sq = self.sq0                    # q spider 
        self.q = np.array(self.q0)            # q
        self.x = self.x0


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
        sqdot = np.zeros((18, 1))
        sq = np.array(self.KinematicChain.stewart_to_spider_q(self.q))

        # Return the position and velocity as python lists.
        return (sq.flatten().tolist(), sqdot.flatten().tolist(), np.array(self.q))

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
