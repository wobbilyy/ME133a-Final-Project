import numpy as np
import math
from TransformHelpers import Rotx, Roty, Rotz
from sympy import symbols, cos, sin, diff

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
        Tx = q[0]-self.center_pos[0]
        Ty = q[1]-self.center_pos[1]
        Tz = q[2]-self.center_pos[2]
        psi = q[3]
        theta = q[4]
        phi = q[5]

        X = [0,0,0,0,0,0] # X matrix
        Y = [0,0,0,0,0,0] # Y matrix
        for i in range(6):
            T = np.array([Tx,Ty,Tz]).transpose()                # Translation of the center of top
            p = np.array([self.top_pos[i][0]-self.center_pos[0], 
                        self.top_pos[i][1]-self.center_pos[1],
                        self.top_pos[i][2]-self.center_pos[2]]).transpose()  # Vector between point on top and center of top
            b = np.array(self.base_pos[i]).transpose()               # Base position of given leg
            Rb = Rotz(psi) @ Roty(theta) @ Rotx(phi)            # Rotation of the top plate
            l = T + Rb @ p - b                                  # Vector of leg
            X[i] = math.sqrt(l[0]**2 + l[1]**2 + l[2]**2)  # Length of leg

            cos_psi = cos(psi)
            sin_psi = sin(psi)
            cos_theta = cos(theta)
            sin_theta = sin(theta)
            cos_phi = cos(phi)
            sin_phi = sin(phi)
            
            T = np.array([Tx,Ty,Tz]).transpose()                # Translation of the center of top
            b = np.array(self.base_pos[i]).transpose()               # Base position of given leg
            R_matrix = np.array([p[0]*(cos_psi*cos_theta) + p[1]*(-sin_psi*cos_phi + cos_psi*sin_theta*sin_phi) + p[2]*(sin_psi*sin_phi + cos_psi*sin_theta*cos_phi), 
                    p[0]*(sin_psi*cos_theta) + p[1]*(cos_psi*cos_phi + sin_psi*sin_theta*sin_phi) + p[2]*(-cos_psi*sin_phi + sin_psi*sin_theta*cos_phi), 
                    p[0]*-sin_theta + p[1]*(cos_theta*sin_phi) + p[2]*(cos_theta*cos_phi)]).transpose()
            l_i = (T + R_matrix - b)

            if np.linalg.norm(Rb @ p - R_matrix.astype(float)) > 0.01:
                print("HELP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            Y[i] = math.sqrt(l_i[0]**2 + l_i[1]**2 + l_i[2]**2)  # Length of leg

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

    ### Why is this not working??? Entries completely mismatch compute_jacobian
    ### Not being used currently.
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


        J_expected = []
        psi_0 = q[3]
        theta_0 = q[4]
        phi_0 = q[5]

        for i in range(6):
            L = X[i]
            p = np.array([self.top_pos[i][0]-self.center_pos[0], 
                        self.top_pos[i][1]-self.center_pos[1],
                        self.top_pos[i][2]-self.center_pos[2]]).transpose()  # Vector between point on top and center of top

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

            
            ######################################## COMPUTER DERIVATIVE ####################################
            psi_1, theta_1, phi_1 = symbols('psi_1 theta_1 phi_1')
            cos_psi = cos(psi_1)
            sin_psi = sin(psi_1)
            cos_theta = cos(theta_1)
            sin_theta = sin(theta_1)
            cos_phi = cos(phi_1)
            sin_phi = sin(phi_1)
            
            R_matrix = [p[0]*(cos_psi*cos_theta) + p[1]*(-sin_psi*cos_phi + cos_psi*sin_theta*sin_phi) + p[2]*(sin_psi*sin_phi + cos_psi*sin_theta*cos_phi), 
                    p[0]*(sin_psi*cos_theta) + p[1]*(cos_psi*cos_phi + sin_psi*sin_theta*sin_phi) + p[2]*(-cos_psi*sin_phi + sin_psi*sin_theta*cos_phi), 
                    p[0]*-sin_theta + p[1]*(cos_theta*sin_phi) + p[2]*(cos_theta*cos_phi)]
            l_i = R_matrix

            subs_dict = {phi_1: phi_0, theta_1: theta_0, psi_1: psi_0}

            dlx_dphi_1 = diff(l_i[0], phi_1).subs(subs_dict)
            
            dlx_dtheta_1 = diff(l_i[0], theta_1).subs(subs_dict)

            dlx_dpsi_1 = diff(l_i[0], psi_1).subs(subs_dict)

            dly_dphi_1 = diff(l_i[1], phi_1).subs(subs_dict)

            dly_dtheta_1 = diff(l_i[1], theta_1).subs(subs_dict)

            dly_dpsi_1 = diff(l_i[1], psi_1).subs(subs_dict)

            dlz_dphi_1 = diff(l_i[2], phi_1).subs(subs_dict)
            
            dlz_dtheta_1 = diff(l_i[2], theta_1).subs(subs_dict)

            dlz_dpsi_1 = diff(l_i[2], psi_1).subs(subs_dict)

            # Total partial derivatives
            dl_dphi_1 = dlx_dphi_1 + dly_dphi_1 + dlz_dphi_1
            dl_dtheta_1 = dlx_dtheta_1 + dly_dtheta_1 + dlz_dtheta_1
            dl_dpsi_1 = dlx_dpsi_1 + dly_dpsi_1 + dlz_dpsi_1
            J_expected.append(np.array([1/L, 1/L, 1/L, float(dl_dpsi_1/L), float(dl_dtheta_1/L), float(dl_dphi_1/L)]))
            
        
        print("JACOBIAN =======================")
        s = [[str(e) for e in row] for row in J]
        lens = [max(map(len, col)) for col in zip(*s)]
        fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
        table = [fmt.format(*row) for row in s]
        print('\n'.join(table))


        print("OTHER JACOBIAN =======================")
        s = [[str(e) for e in row] for row in J_expected]
        lens = [max(map(len, col)) for col in zip(*s)]
        fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
        table = [fmt.format(*row) for row in s]
        print('\n'.join(table))
        

        return J_expected

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
            J = self.compute_jacobian(q) 
            #J_wrong = self.invJac(q)

            # Compute the delta and adjust.
            xdelta = np.array(np.array(xgoal) - np.array(x)).transpose()
            #print(f"   our lengths: {x} \nvs desired lengths: {xgoal}")
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
    print(f"Starting leg lengths: {K.invkin(q_start)}")

    test_x = [55.8558,62.5313,52.7436,55.1457,44.7972,51.9910]
    # There are a variety of valid positions for this.
    q = K.fkin(test_x, q_start)
    print(f"For leg lengths {test_x}: found q = {q}")
    print(f"Leg lengths given q: {K.invkin(q)}")
    