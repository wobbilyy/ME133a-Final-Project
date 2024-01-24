'''
This file contains the relevant kinematic chain helper functions to get the
correct platform orientation and positions, and also the correct leg lengths.

Included are:
    invkin             : Computes the inverse kinematics given platform position. 
    compute_jacobian   : Computes the Jacobian using numerical iteration.
    fkin               : Computes the forward kinematics given leg lengths.
    stewart_to_spider_q: Converts the platform position to URDF leg joint values.
    get_leg_vectors    : Converts the platform position to leg vectors.
'''
import numpy as np
import math

from finalprojectcode.TransformHelpers import Rotx, Roty, Rotz

class KinematicChain():
    def __init__(self, PLATFORM_MOUNTS_POS, PLATFORM_CENTER, BASE_MOUNTS_POS):
        # Set up initial positions
        self.PLATFORM_MOUNTS_POS = PLATFORM_MOUNTS_POS
        self.PLATFORM_CENTER = PLATFORM_CENTER
        self.BASE_MOUNTS_POS = BASE_MOUNTS_POS
        
        self.lam = 20   # For Newton Raphson

    def invkin(self, q):
        '''
        Calculates the inverse kinematics for the leg lengths given the platform 
        position and orientation.

        Arguments:
            (List) q : platform position and orientation [Tx, Ty, Tz, psi, theta, phi]
        Returns:
            (List) x : leg lengths [L1, L2, L3, L4, L5, L6]
        '''
        leg_vectors = self.get_leg_vectors(q)

        X = [0,0,0,0,0,0] # X matrix
        for i in range(6):
            l = leg_vectors[i]
            X[i] = math.sqrt(l[0]**2 + l[1]**2 + l[2]**2)  # Length of leg

        return X

    def compute_jacobian(self, q, epsilon=1e-6):
        '''
        Numerically compute Jacobian without derivatives. 
        J = (fkin(x) + fkin(x+q))/ dq
        
        Arguments:
            (List) q : platform position and orientation [Tx, Ty, Tz, psi, theta, phi]
        Returns:
            (6x6 np array) J: the Jacobian from each leg
        '''
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
        Numerial Newton Raphson iteration with invkin to calculate forward 
        kinematics for platform position and orientation given leg lengths

        Arguments:
            (List) x : leg lengths [L1, L2, L3, L4, L5, L6]
            (List) qstart: platform location guess [Tx, Ty, Tz, psi, theta, phi]
        Returns:
            (List) q : platform position and orientation [Tx, Ty, Tz, psi, theta, phi] 
        '''
        # Set the initial joint value guess.
        q = qstart

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
            qdelta = np.linalg.pinv(J) @ xdelta
            q = q + qdelta

            # Check whether to break.
            if np.linalg.norm(np.array(xgoal) - np.array(x)) < 1e-12:
                return q
        
        # failed to converge, return initial q
        return q
    
    def stewart_to_spider_q(self, stewart_q):
        '''
        Converts the platform position and orientation to leg joint values.

        Arguments:
            (List) q : platform position and orientation [Tx, Ty, Tz, psi, theta, phi]
        Returns:
            (List) spider_q [leg1-pitch, leg1-roll, leg1-prismatic,
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

            pitch = np.arctan2(dz,r)       # Rotation about Y 
            roll =  np.arctan2(-dx/r,dy/r) # Rotation about X

            spider_q.append(pitch)
            spider_q.append(roll)
            spider_q.append(leg_length - 2.4)

        return spider_q

    def get_leg_vectors(self, q):
        '''
        Get the leg vectors given the platform position and orientation.
        Arguments:
            (List) q : platform position and orientation [Tx, Ty, Tz, psi, theta, phi]        
        Returns:
            (List) x : leg vectors [l1x, l1y, l1z, 
                                    l2x, l2y, l2z,
                                    l3x, l3y, l3z,
                                    l4x, l4y, l4z,
                                    l5x, l5y, l5z,
                                    l6x, l6y, l6z]
        '''
        Tx = q[0] 
        Ty = q[1] 
        Tz = q[2]
        psi = q[3]
        theta = q[4]
        phi = q[5]

        L = []

        for i in range(6):
            T = np.array([Tx,Ty,Tz]).transpose()                                                  # Translation of platform from origin
            p = np.array([self.PLATFORM_MOUNTS_POS[i][0] - self.PLATFORM_CENTER[0], 
                          self.PLATFORM_MOUNTS_POS[i][1] - self.PLATFORM_CENTER[1],
                          self.PLATFORM_MOUNTS_POS[i][2] - self.PLATFORM_CENTER[2]]).transpose()  # Vector from platform center to leg mount point
            b = np.array(self.BASE_MOUNTS_POS[i]).transpose()                                     # Vector from base mount point to leg mount point
            Rb = Rotx(psi) @ Roty(theta) @ Rotz(phi)                                              # Rotation of platform 
            l = (T + Rb @ p - b)                                                                  # Leg vector
            L.append([float(l[0]), float(l[1]), float(l[2])])
        
        return L
