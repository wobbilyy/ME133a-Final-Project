import numpy as np
import math
from TransformHelpers import Rotx, Roty, Rotz, eR

# Let q = [Tx, Ty, Tz, psi, theta, phi]
# x = [L1, L2, L3, L4, L5]
class KinematicHelpers():
    def __init__(self, top_pos, center_pos, base_pos):
        # Set up initial positions
        self.top_pos = top_pos
        self.center_pos = center_pos
        self.base_pos = base_pos
        
        self.lam = 20   # For Newton Raphson

    def invkin(self, q):
        '''
        Given q = [Tx, Ty, Tz, psi, theta, phi],
        fkin(q) returns x = [L1, L2, L3, L4, L5, L6]
        '''
        print(self)
        Tx = q[0]-self.center_pos[0]
        Ty = q[1]-self.center_pos[1]
        Tz = q[2]-self.center_pos[2]
        psi = q[3]
        theta = q[4]
        phi = q[5]

        X = [0,0,0,0,0,0] # X matrix
        for i in range(6):
            T = np.array([Tx,Ty,Tz]).transpose()                # Translation of the center of top
            p = np.array([self.top_pos[i][0]-self.center_pos[0], 
                        self.top_pos[i][1]-self.center_pos[1],
                        self.top_pos[i][2]-self.center_pos[2]]).transpose()  # Vector between point on top and center of top
            b = np.array(self.base_pos[i]).transpose()               # Base position of given leg
            Rb = Rotz(psi) @ Roty(theta) @ Rotx(phi)            # Rotation of the top plate
            l = T + Rb @ p - b                                  # Vector of leg
            X[i] = math.sqrt(l[0]**2 + l[1]**2 + l[2]**2)  # Length of leg

        return X



    def invJac(self, q):
        '''Given Given q = [Tx, Ty, Tz, psi, theta, phi],
        Jaq(q) returns J(q) s.t. J(q)*q'= dx/dt'''
        Tx = q[0]
        Ty = q[1]
        Tz = q[2]
        psi = q[3]
        theta = q[4]
        phi = q[5]

        X = self.invkin(q)
        J = [] # Fill below
        
        for i in range(6):
            L = X[i]
            p = [self.top_pos[i][0]-self.center_pos[0], 
                self.top_pos[i][1]-self.center_pos[1],
                self.top_pos[i][2]-self.center_pos[2]]
            
            # Partial derivatives (ugly)
            '''
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
            '''

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

            dlz_dtheta = 0
            dlz_dpsi = 0
            # Total partial derivatives
            dl_dphi = dlx_dphi + dly_dphi + dlz_dphi
            dl_dtheta = dlx_dtheta + dly_dtheta + dlz_dtheta
            dl_dpsi = dlx_dpsi + dly_dpsi + dlz_dpsi
            J.append(np.array([1/L, 1/L, 1/L, dl_dpsi/L, dl_dtheta/L, dl_dphi/L]))
        
        return np.array(J)


    def fkin(self, xgoal, qstart):
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
        q = qstart

        # Get goal translation and rotation.
        
        # Number of steps to try.
        N = 100

        # Iterate
        for i in range(N+1):
            # Determine where you are
            x = self.invkin(q)
            J = self.invJac(q)

            # Compute the delta and adjust.
            xdelta = np.array(np.array(xgoal) - np.array(x)).transpose()
            print(f"   our lengths: {x} \nvs desired lengths: {xgoal}")
            print(J)
            qdelta = np.linalg.pinv(J) @ xdelta
            q = q + qdelta

            # Check whether to break.
            if np.linalg.norm(np.array(xgoal) - np.array(x)) < 1e-12:
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

#
#  Testing kinematics helpers
#
if __name__ == "__main__":
    
    # Test case from https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7942377&tag=1
    
    base_pos = [[28.9778,7.7646,0],
               [-7.7646,28.9778,0],
               [-21.2132,21.2132,0],
               [-21.2132,-21.2132,0],
               [-7.7646,-28.9778,0],
               [28.9778,-7.7646,0]]
    center_pos = [0,0,40]
    top_pos = [[14.1421,14.1421,40],
                [5.1764,19.3185,40],
                [-19.3185,5.1764,40],
                [-19.3185,-5.1731,40],
                [5.1764,-19.3185,40],
                [14.1421,-14.1421,40]]
    q_start = [0,0,0,0,0,0]

    K = KinematicHelpers(top_pos, center_pos, base_pos)
    # What are our current leg lengths?
    print(K.invkin(q_start))

    test_x = [55.8558,62.5313,52.7436,55.1457,44.7972,51.9910]
    # There are a variety of valid positions for this.
    print(K.fkin(test_x, q_start))
    