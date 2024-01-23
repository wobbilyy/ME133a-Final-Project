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

# Grab the general fkin from HW5 P5.
from finalprojectcode.KinematicChain     import KinematicChain
from finalprojectcode.Sliders import Sliders

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
        self.base_pos = [[ 0.25     ,  1.5       , 0],
                    [-0.25     ,  1.5       , 0],
                    [-1.424038 , -0.533494  , 0],
                    [-1.1174038, -0.96650635, 0], 
                    [ 1.1174038, -0.96650635, 0], 
                    [ 1.424038 , -0.533494  , 0]]
        self.center_pos = [0        ,  0         , height - 0.2]
        self.top_pos = [[r * np.cos(np.radians(0))  , r * np.sin(np.radians(0))  , height - 0.2],
                   [r * np.cos(np.radians(60)) , r * np.sin(np.radians(60)) , height - 0.2],
                   [r * np.cos(np.radians(120)), r * np.sin(np.radians(120)), height - 0.2],
                   [r * np.cos(np.radians(180)), r * np.sin(np.radians(180)), height - 0.2],
                   [r * np.cos(np.radians(240)), r * np.sin(np.radians(240)), height - 0.2],
                   [r * np.cos(np.radians(300)), r * np.sin(np.radians(300)), height - 0.2]] 

        self.K = Sliders(self.top_pos, self.center_pos, self.base_pos, [0,0,0,0,0,0])

        # Define the various points.
        self.qgoal = [0,0,0,0,0,0]      # Desired top orientation platform
        self.xgoal = self.K.invkin()    # Desired leg lengths
        self.spiderq = self.K.stewart_to_spider_q()
        
        
        self.KinematicChain = KinematicHelpers(self.top_pos, self.center_pos, self.base_pos)
        # # Define the various points.
        self.x0 = [0, 0, 0, np.radians(0), np.radians(0), np.radians(0)]
        self.q0 = self.KinematicChain.stewart_to_spider_q(self.x0)
        
        print(f"!!!!For leg lengths {self.xgoal}: found q = {self.qgoal}, spider q = {self.q0}")
        print("!!!!!================================================================")

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
        self.spiderq = self.K.stewart_to_spider_q()
        q = np.array(self.spiderq)
        print(f"For leg lengths {self.xgoal}: found q = {self.qgoal}, spider q = {q}")
        print("================================================================")

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist(), self.qgoal)


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
    
    generator.run_gui()
    
    
    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
