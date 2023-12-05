'''
final project
'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TransformHelpers   import *
from hw5code.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Define the various points.
        self.q0 = np.radians(np.array([0, 90, 0, -90, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1,1))
        self.R0 = Reye()

        self.low_right  = np.array([-0.3, 0.5, 0.15]).reshape((-1,1))
        self.low_left  = np.array([0.3, 0.5, 0.15]).reshape((-1,1))
        self.phigh = np.array([0.0, 0.5, 0.9]).reshape((-1,1))

        # Initialize the current/starting joint position.
        self._lambda = 20
        self.q  = self.q0

        # self.qgoal = np.array([-pi / 4, -pi / 4, pi / 2, -pi / 2, 0, 0, 0]).reshape((-1,1))
        # self.lambda_s = 100
        self.c_repulse = 1


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        # return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']
        return ['theta1', 'theta2', 'd3']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):    
        return None
        if t > 8:
            return None

        if t < 3.0: 
            (s0, s0dot) = goto(t, 3.0, 0.0, 1.0)

            pd = self.p0 + (self.low_right - self.p0) * s0
            vd =           (self.low_right - self.p0) * s0dot

            Rd = self.R0
            wd = np.zeros((3,1))

        else:
            t1 = (t - 3) % 5

            if t1 < 5 / 2:
                if t1 < 2.5 / 2:
                    (sp, spdot) = spline(t1, 2.5 / 2, 0, 1.0, 0.0, 0.0)
                    pd = self.low_right + (self.phigh - self.low_right) * sp
                    vd =                 (self.phigh - self.low_right) * spdot
                else:
                    (sp, spdot) = spline(t1 - 2.5 / 2, 2.5 / 2, 0, 1.0, 0.0, 0.0)
                    pd = self.phigh + (self.low_left - self.phigh) * sp
                    vd =              (self.low_left - self.phigh) * spdot
                
                (sR, sRdot) = goto(t1, 5.0 / 2, 0, 1.0)
            else:
                if t1 < 7.5 / 2:
                    (sp, spdot) = spline(t1 - 5.0 / 2, 2.5 / 2, 0, 1.0, 0.0, 0.0)
                    pd = self.low_left + (self.phigh - self.low_left) * sp
                    vd =                 (self.phigh - self.low_left) * spdot
                else:
                    (sp, spdot) = spline(t1 - 7.5 / 2, 2.5 / 2, 0, 1.0, 0.0, 0.0)
                    pd = self.phigh + (self.low_right - self.phigh) * sp
                    vd =              (self.low_right - self.phigh) * spdot
                
                (sR, sRdot) = goto(t1 - 5.0 / 2, 5.0 / 2, 1.0, 0.0)

            Rd = Roty(-pi / 2 * sR) @ Rotz(pi / 2 * sR)
            wd = ey() * (-pi / 2 * sRdot) + Roty(-pi / 2 * sR) @ ez() * (pi / 2 * sRdot)
    
        ptip, Rtip, Jv, Jw = self.chain.fkin(self.q)

        # J = np.vstack( (np.vstack((Jv, Jw)), np.array([0.5, 0, 1, 0, 0, 0, 0])) )
        # qdot = np.linalg.pinv(J) @ ( np.vstack( (np.vstack((vd, wd)), 0) ) + self._lambda * np.vstack( (np.vstack((ep(pd, ptip), eR(Rd, Rtip))), -(0.5 * self.q[0] + self.q[2]) ) ) )

        J = np.vstack((Jv, Jw))
        qdot = np.linalg.pinv(J) @ (np.vstack((vd, wd)) + self._lambda * np.vstack((ep(pd, ptip), eR(Rd, Rtip))))

        qdot_s = np.array([ self.q[0], max(abs(self.q[0]), self.q[1]), [0], [0], [0], [0], [0] ])
        qdot_s = self.c_repulse * (1 / (self.q[0] ** 2 + self.q[1] ** 2)) * qdot_s

        qdot = qdot + ((np.identity(7) - np.linalg.pinv(J) @ J) @ qdot_s)

        self.q += qdot * dt
        q = self.q

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
