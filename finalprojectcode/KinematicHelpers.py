import numpy as np
import math
from TransformHelpers import Rotx, Roty, Rotz, eR

# Let q = [Tx, Ty, Tz, psi, theta, phi]
# x = [L1, L2, L3, L4, L5]
class KinematicHelpers():
    def __init__(self):
        # Set up initial positions
        self.top_pos = [[0,0,0],
                        [0,0,0],
                        [0,0,0],
                        [0,0,0],
                        [0,0,0],
                        [0,0,0]]
        self.center_pos = [0,0,0]
        self.base_pos = [[0,0,0],
                        [0,0,0],
                        [0,0,0],
                        [0,0,0],
                        [0,0,0],
                        [0,0,0]]
        
        self.lam = 20   # For Newton Raphson

    def invkin(q, self):
        '''
        Given q = [Tx, Ty, Tz, psi, theta, phi],
        fkin(q) returns x = [L1, L2, L3, L4, L5, L6]
        '''
        Tx = q[0]
        Ty = q[1]
        Tz = q[2]
        psi = q[3]
        theta = q[4]
        phi = q[5]

        X = [0,0,0,0,0,0] # X matrix
        for i in range(6):
            T = np.array([Tx,Ty,Tz]).transpose()                # Translation of the center of top
            p = np.array([self.top_pos[0]-self.center_pos[0], 
                        self.top_pos[1]-self.center_pos[1],
                        self.top_pos[2]-self.center_pos[2]]).transpose()  # Vector between point on top and center of top
            b = np.array(self.base_pos[i]).transpose()               # Base position of given leg
            Rb = Rotz(psi) @ Roty(theta) @ Rotx(phi)            # Rotation of the top plate
            l = T + Rb @ p - b                                  # Vector of leg
            X[i] = math.sqrt(l[0][0]**2 + l[0][1]**2 + l[0][2]**2)  # Length of leg

        return X



    def invJac(q, self):
        '''Given Given q = [Tx, Ty, Tz, psi, theta, phi],
        Jaq(q) returns J(q) s.t. J(q)*q'= dx/dt'''
        Tx = q[0]
        Ty = q[1]
        Tz = q[2]
        psi = q[3]
        theta = q[4]
        phi = q[5]

        X = KinematicHelpers.invkin(q)
        J = np.array([]) # Fill below
        
        for i in range(6):
            L = X[i]
            p = [self.top_pos[0]-self.center_pos[0], 
                self.top_pos[1]-self.center_pos[1],
                self.top_pos[2]-self.center_pos[2]]
            
            # Partial derivatives (ugly)
            dlx_dphi = (np.sin(psi)*np.sin(phi)+np.cos(psi)*np.sin(theta)*np.cos(phi))*p[1] 
            + (np.sin(psi)*np.cos(phi)-np.cos(psi)*np.sin(theta)*np.sin(phi))*p[2]
            
            dlx_dtheta = (-np.cos(psi)*np.sin(theta))*p[0]
            + (np.cos(psi)*np.cos(theta)*np.sin(phi))*p[1]
            + (np.cos(psi)*np.cos(theta)*np.cos(phi))*p[2]

            dlx_dpsi = (-np.sin(psi)*np.cos(theta))*p[0]
            + (-np.cos(psi)*np.cos(phi)-np.sin(psi)*np.sin(theta)*np.sin(phi))*p[1]
            + (np.cos(psi)*np.sin(phi)-np.sin(psi)*np.sin(theta)*np.cos(phi))*p[2]

            dly_dphi = (-np.cos(psi)*np.sin(phi)+np.sin(psi)*np.sin(theta)*np.cos(phi))*p[1]
            + (-np.cos(psi)*np.cos(phi)-np.sin(psi)*np.sin(theta)*np.sin(psi))*p[2]

            dly_dtheta = (-np.sin(psi)*np.sin(theta))*p[0]
            +(np.sin(psi)*np.cos(theta)*np.sin(phi))*p[1]
            +(np.sin(psi)*np.cos(theta)*np.cos(phi))*p[2]

            dly_dpsi = (np.cos(psi)*np.cos(theta))*p[0]
            +(-np.sin(psi)*np.cos(phi)+np.cos(psi)*np.sin(theta)*np.sin(psi))*p[1]
            +(np.sin(psi)*np.sin(phi)+np.cos(psi)*np.sin(theta)*np.cos(phi))*p[2]

            dlz_dphi = (np.cos(theta)*np.cos(phi))*p[1]
            +(-np.cos(theta)*np.sin(phi))*p[2]

            dlz_dtheta = (-np.cos(theta))*p[0]
            +(-np.sin(theta)*np.sin(phi))*p[1]
            +(-np.sin(theta)*np.cos(phi))*p[2]

            dlz_dpsi = 0

            # Total partial derivatives
            dl_dphi = dlx_dphi + dly_dphi + dlz_dphi
            dl_dtheta = dlx_dtheta + dly_dtheta + dlz_dtheta
            dl_dpsi = dlx_dpsi + dly_dpsi + dlz_dpsi
            J.append(np.array([1/L, 1/L, 1/L, dl_dpsi/L, dl_dtheta/L, dl_dphi/L]))
        
        return J


    def fkin(xgoal, qstart):
        '''
        Newton Raphson with invkin and invJac to find forward kinematics
        of given leg lengths
        Used hw5p2 solution code as template.

        Given x = [L1, L2, L3, L4, L5, L6],
        and qstart = [Tx, Ty, Tz, psi, theta, phi] (intial guess)
        fkin returns q = [Tx, Ty, Tz, psi, theta, phi] (resulting orientation of top platform)

        FIXME: will this work?
        '''

        # Set the initial joint value guess.
        q = np.array(qstart).reshape(6,1)
        
        # Number of steps to try.
        N = 100

        # Iterate
        for i in range(N+1):
            # Determine where you are
            x = KinematicHelpers.invkin(q)
            J = KinematicHelpers.invJac(q)

            # Compute the delta and adjust.
            xdelta = (xgoal - x)
            qdelta = np.linalg.pinv(J) @ xdelta
            q = q + qdelta

            # Check whether to break.
            if np.linalg.norm(x-xgoal) < 1e-12:
                break
        
        return q
    

    def stewart_to_spider_q(stewart_q):
        '''
        In: stewart_q [Tx, Ty, Tz, psi, theta, phi]
        Out: spider_q [??]
        '''
        print("Need to do")

        '''
        Inverse kinematics for trajectories (from HW 6)

        # Compute the inverse kinematics
        vr = vd + self.lam * ep(pdlast, p)
        wr = wd + self.lam * eR(Rdlast, R)
        J = np.vstack((Jv, Jw))
        xrdot = np.vstack((vr, wr))
        qdot = np.linalg.inv(J) @ xrdot

        # Compute the inverse kinematics
        # Integrate the joint position.
        q = qlast + dt * qdot

        # Save the joint value and desired values for next cycle.
        self.q = q
        self.pd = pd
        self.Rd = Rd
        '''