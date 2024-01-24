import numpy as np
import math
from sympy import symbols, cos, sin, diff

from finalprojectcode.TransformHelpers import Rotx, Roty, Rotz

# Let q = [Tx, Ty, Tz, psi, theta, phi]
# x = [L1, L2, L3, L4, L5]
class KinematicChain():
    def __init__(self, top_pos, center_pos, base_pos):
        # Set up initial positions
        self.top_pos = top_pos
        self.center_pos = center_pos
        self.base_pos = base_pos
        
        self.lam = 20   # For Newton Raphson

    def invkin(self, q):
        '''
        Given q = [Tx, Ty, Tz, psi, theta, phi],
        invkin(q) returns x = [L1, L2, L3, L4, L5, L6]
        '''
        leg_vectors = self.get_leg_vectors(q)

        X = [0,0,0,0,0,0] # X matrix
        for i in range(6):
            l = leg_vectors[i]
            X[i] = math.sqrt(l[0]**2 + l[1]**2 + l[2]**2)  # Length of leg

        return X


    # Numerically compute Jacobian without derivatives. Just do J = (fkin(x) + fkin(x+q))/ dq
    def compute_jacobian(self, q, epsilon=1e-6):
        J = np.zeros((6, 6))
        X0 = self.invkin(q)

        for i in range(6):
            q_perturbed = q.copy()
            q_perturbed[i] += epsilon

            X_perturbed = self.invkin(q_perturbed)
            numerical_derivative = (np.array(X_perturbed) - np.array(X0)) / epsilon

            J[:, i] = numerical_derivative

        return J

    def fkin(self, xgoal, qstart):
        '''
        Newton Raphson with invkin and invJac to find forward kinematics
        of given leg lengths
        Used hw5p2 solution code as template.

        Given x = [L1, L2, L3, L4, L5, L6],
        and qstart = [Tx, Ty, Tz, psi, theta, phi]     (intial guess)
        fkin returns q = [Tx, Ty, Tz, psi, theta, phi] (resulting orientation of top platform)
        '''

        # Set the initial joint value guess.
        q = qstart

        # Get goal translation and rotation.
        
        # Number of steps to try.
        N = 20

        # Iterate
        for i in range(N + 1):
            # Determine where you are
            x = self.invkin(q)
            J = self.compute_jacobian(q) 
            #J_wrong = self.invJac(q)

            # Compute the delta and adjust.
            xdelta = np.array(np.array(xgoal) - np.array(x)).transpose()
            #print(f"   our lengths: {x} \nvs desired lengths: {xgoal}")
            qdelta = np.linalg.pinv(J) @ xdelta
            q = q + qdelta

            # Check whether to break.
            if np.linalg.norm(np.array(xgoal) - np.array(x)) < 1e-12:
                return q
        
        # failed to converge, return initial q
        return q
    
    def stewart_to_spider_q(self, stewart_q):
        '''
        In: stewart_q [Tx, Ty, Tz, psi, theta, phi]
        Out: spider_q [leg1-pitch, leg1-roll, leg1-prismatic,
                leg2-pitch, leg2-roll, leg2-prismatic,
                leg3-pitch, leg3-roll, leg3-prismatic,
                leg4-pitch, leg4-roll, leg4-prismatic,
                leg5-pitch, leg5-roll, leg5-prismatic,
                leg6-pitch, leg6-roll, leg6-prismatic]
        '''
        
        # To get the leg_pitch and leg_roll, let's use our 2DOF inverse kinematics
        # for a pan-tilt algorithm
        leg_vectors = self.get_leg_vectors(stewart_q)
        leg_lengths = self.invkin(stewart_q)
        spider_q = []

        for i in range(6):
            leg = leg_vectors[i]
            leg_length = leg_lengths[i]
            R = np.sqrt(leg[0]**2 + leg[1]**2 + leg[2]**2)
            dx = leg[1]/R
            dy = leg[2]/R
            dz = leg[0]/R
            r = np.sqrt(dx**2 + dy**2)

            pitch = np.arctan2(dz,r) # Rotation about Y 
            roll =  np.arctan2(-dx/r,dy/r) # Rotation about X

            spider_q.append(pitch)
            spider_q.append(roll)
            spider_q.append(leg_length - 2.4)

        return spider_q

    def get_leg_vectors(self, q):
        '''
        Given q = [Tx, Ty, Tz, psi, theta, phi],
        get_leg_vectors(q) returns x = [l1x, l1y, l1z, 
                                        l2x, l2y, l2z,
                                        l3x, l3y, l3z,
                                        l4x, l4y, l4z,
                                        l5x, l5y, l5z,
                                        l6x, l6y, l6z]
        '''
        Tx = q[0] + self.center_pos[0]
        Ty = q[1] + self.center_pos[1]
        Tz = q[2] + self.center_pos[2]
        psi = q[3]
        theta = q[4]
        phi = q[5]

        L = []

        for i in range(6):
            T = np.array([Tx,Ty,Tz]).transpose()                # Translation of the center of top
            p = np.array([self.top_pos[i][0] - self.center_pos[0], 
                          self.top_pos[i][1] - self.center_pos[1],
                          self.top_pos[i][2] - self.center_pos[2]]).transpose()  # Vector between point on top and center of top
            b = np.array(self.base_pos[i]).transpose()               # Base position of given leg
            Rb = Rotx(psi) @ Roty(theta) @ Rotz(phi)            # Rotation of the top plate
            l = (T + Rb @ p - b)                                # Vector of leg
            L.append([float(l[0]), float(l[1]), float(l[2])])
        
        return L
