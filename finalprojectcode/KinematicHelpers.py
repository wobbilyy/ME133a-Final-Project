import numpy as np
import math
from TransformHelpers import Rotx, Roty, Rotz
from sympy import symbols, cos, sin, diff
import tkinter as tk

# Let q = [Tx, Ty, Tz, psi, theta, phi]
# x = [L1, L2, L3, L4, L5]
class KinematicHelpers():
    def __init__(self, top_pos, center_pos, base_pos, start_q):
        # Set up initial positions
        # We never update top_pos, center_pos, base_pos, because all we care about is the
        # relative position of top_pos relative to center_pos
        self.top_pos = top_pos
        self.center_pos = center_pos
        self.base_pos = base_pos

        self.q = start_q    # q for stewart platform
                            # q = [Tx, Ty, Tz, psi, theta, phi]

    def invkin(self,q=[]):
        '''
        Given self.q = [Tx, Ty, Tz, psi, theta, phi],
        invkin(q) returns x = [L1, L2, L3, L4, L5, L6]
        '''
        if len(q) < 6: q = self.q
        leg_vectors = self.get_leg_vectors(q)

        X = [0,0,0,0,0,0] # X matrix
        for i in range(6):
            l = leg_vectors[i]
            X[i] = math.sqrt(l[0]**2 + l[1]**2 + l[2]**2)  # Length of leg

        return X


    # Numerically compute Jacobian without derivatives. Just do J = (fkin(x) + fkin(x+q))/ dq
    def compute_jacobian(self, q=[], epsilon=1e-6):
        if q == []: q = self.q
        J = np.zeros((6, 6))
        X0 = self.invkin()

        for i in range(6):
            q_perturbed = self.q.copy()
            q_perturbed[i] += epsilon

            X_perturbed = self.invkin(q_perturbed)
            numerical_derivative = (np.array(X_perturbed) - np.array(X0)) / epsilon

            J[:, i] = numerical_derivative

        return J

    ### Why is this not working??? Entries completely mismatch compute_jacobian
    ### Not being used currently.
    def invJac(self):
        '''Given Given q = [Tx, Ty, Tz, psi, theta, phi],
        Jaq(q) returns J(q) s.t. J(q)*q'= dx/dt'''
        q=self.q
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

            ######################################## HAND-CALCULATED DERIVATIVE ####################################
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

            
            ######################################## COMPUTER-CALCULATED DERIVATIVE ####################################
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
            
        
        '''Debugging of derivatives of fkin matrix
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
        '''
        

        return J_expected

    def fkin(self, xgoal, qstart = []):
        '''
        Newton Raphson with invkin and invJac to find forward kinematics
        of given leg lengths
        Used hw5p2 solution code as template.

        Given xgoal = [L1, L2, L3, L4, L5, L6],
        and self.q = [Tx, Ty, Tz, psi, theta, phi] (intial guess)
        fkin returns q = [Tx, Ty, Tz, psi, theta, phi] (resulting orientation of top platform)
        
        Also UPDATES self.q
        '''

        # Set the initial joint value guess.
        if qstart == []: qstart = self.q
        q = qstart
        q_0 = q

        # Get goal translation and rotation.
        
        # Number of steps to try.
        N = 100

        # Iterate
        for i in range(N+1):
            # Determine where you are
            x = self.invkin()
            J = self.compute_jacobian() 
            #J_wrong = self.invJac(q)

            # Compute the delta and adjust.
            xdelta = np.array(np.array(xgoal) - np.array(x)).transpose()
            #print(f"   our lengths: {x} \nvs desired lengths: {xgoal}")
            qdelta = np.linalg.pinv(J) @ xdelta
            q = q + qdelta
            self.q = q

            # Check whether to break.
            if np.linalg.norm(np.array(xgoal) - np.array(x)) < 1e-12:
                return q
        
        # Failure to converge. Return to original configuration.
        self.q = q_0
        q = q_0
        print(f"Failed to converge within {N} steps")
        return q


    def stewart_to_spider_q(self):
        '''
        In: self.q = stewart_q [Tx, Ty, Tz, psi, theta, phi]
        Out: spider_q [leg1-pitch, leg1-roll, leg1-prismatic,
                leg2-pitch, leg2-roll, leg2-prismatic,
                leg3-pitch, leg3-roll, leg3-prismatic,
                leg4-pitch, leg4-roll, leg4-prismatic,
                leg5-pitch, leg5-roll, leg5-prismatic,
                leg6-pitch, leg6-roll, leg6-prismatic]
        '''
        
        # To get the leg_pitch and leg_roll, let's use our 2DOF inverse kinematics
        # for a pan-tilt algorithm
        stewart_q = self.q
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

            # Let's check. Do this pitch and roll work out?
            # Does the "apparent" leg calculated from pitch and roll
            # match the actual leg we were trying to find the angles of?
            p_0 = (np.array([0,0,R]).transpose())
            apparent_leg = Rotx(roll) @ Roty(pitch) @ p_0
            if np.linalg.norm(apparent_leg - leg) > 0.1:
                print("error: apparent leg and actual leg differ")

            spider_q.append(pitch)
            spider_q.append(roll)
            spider_q.append(leg_length)

        return spider_q


    def get_leg_vectors(self,q):
        '''
        Given q = [Tx, Ty, Tz, psi, theta, phi],
        get_leg_vectors(q) returns x = [l1x, l1y, l1z, 
                                        l2x, l2y, l2z,
                                        l3x, l3y, l3z,
                                        l4x, l4y, l4z,
                                        l5x, l5y, l5z,
                                        l6x, l6y, l6z]
        '''

        Tx = q[0]+self.center_pos[0]
        Ty = q[1]+self.center_pos[1]
        Tz = q[2]+self.center_pos[2]
        psi = q[3]
        theta = q[4]
        phi = q[5]

        L = []

        for i in range(6):
            T = np.array([Tx,Ty,Tz]).transpose()                # Translation of the center of top
            p = np.array([self.top_pos[i][0]-self.center_pos[0], 
                        self.top_pos[i][1]-self.center_pos[1],
                        self.top_pos[i][2]-self.center_pos[2]]).transpose()  # Vector between point on top and center of top
            b = np.array(self.base_pos[i]).transpose()               # Base position of given leg
            Rb = Rotz(psi) @ Roty(theta) @ Rotx(phi)            # Rotation of the top plate
            l = (T + Rb @ p - b)                                  # Vector of leg
            L.append([float(l[0]), float(l[1]), float(l[2])])
        
        return L

#
#  Testing kinematics helpers
#
if __name__ == "__main__":
    
    '''
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

    K = KinematicHelpers(top_pos, center_pos, base_pos, q_start)
    # What are our current leg lengths?
    print(f"Starting leg lengths: {K.invkin()} and orientation: {K.q}")

    test_x = [55.8558,62.5313,52.7436,55.1457,44.7972,51.9910]
    # There are a variety of valid positions for this.
    q = K.fkin(test_x)
    print(f"For leg lengths {test_x}: found q = {K.q}")
    print(f"Leg lengths given q: {K.invkin()}")

    test_x = [43.136649159850144, 43.136661408713586, 43.13662993709639, 43.137856878848304, 43.136661408713586, 43.136649159850144]
    # Should go back to starting position
    q = K.fkin(test_x)
    print(f"For leg lengths {test_x}: found q = {q}")
    print(f"Leg lengths given q: {K.invkin()}")
    #'''

    ########### TEST: Actual stewart dimensions
    # Initialize kinematic chain helper object
    base_pos = [[0.25, 1.5, 0],
                [-0.25, 1.5, 0],
                [-1.424038, -0.533494, 0], 
                [-1.1174038, -0.96650635, 0], 
                [1.1174038, -0.966506, 0], 
                [1.424038, -0.533494, 0]]
    r =  1.25
    height = 2.6
    center_pos = [0,0,height-0.2]
    top_pos = [
            [r * np.cos(np.pi/180*(0)), r * np.sin(np.pi/180*(0)), height - 0.2],
            [r * np.cos(np.pi/180*(60)), r * np.sin(np.pi/180*(60)), height - 0.2],
            [r * np.cos(np.pi/180*(120)), r * np.sin(np.pi/180*(120)), height - 0.2],
            [r * np.cos(np.pi/180*(180)), r * np.sin(np.pi/180*(180)), height - 0.2],
            [r * np.cos(np.pi/180*(240)), r * np.sin(np.pi/180*(240)), height - 0.2],
            [r * np.cos(np.pi/180*(300)), r * np.sin(np.pi/180*(300)), height - 0.2]
            ]   
    start_q = [0,0,0,0,0,0]
    K = KinematicHelpers(top_pos, center_pos, base_pos, start_q)
    # What are our current leg lengths?
    xgoal = K.invkin()
    xgoal_0 = K.invkin()
    '''
    print(f"ACTUAL: Starting leg lengths: {xgoal}")
    print(f"For leg lengths {xgoal}: found q = {K.fkin(xgoal)}")
    print(f"Spider q: {K.stewart_to_spider_q()}")
    #'''
    #'''
        # Now, using K, we shall attempt to change the leg lengths with sliders and view the output.
    def update_values():
        xgoal = [slider1.get(), slider2.get(), slider3.get(), slider4.get(), slider5.get(), slider6.get()]
        q_new = K.fkin(xgoal)
        print(f"For leg lengths {xgoal}: found q = {q_new}")
        print("================================================================")
        if (np.linalg.norm(np.array(xgoal) - np.array(K.invkin())) < 0.01):
            # Congrats, we actually converged. Update slider values.
            label1.config(text=f"Leg 1: {slider1.get():.2f}")
            label2.config(text=f"Leg 2: {slider2.get():.2f}")
            label3.config(text=f"Leg 3: {slider3.get():.2f}")
            label4.config(text=f"Leg 4: {slider4.get():.2f}")
            label5.config(text=f"Leg 5: {slider5.get():.2f}")
            label6.config(text=f"Leg 6: {slider6.get():.2f}")
        else:
            # Didn't converge. Set sliders to the last valid value.
            xgoal = K.invkin()
            var1.set(float("{:.2f}".format(xgoal[0])))
            var2.set(float("{:.2f}".format(xgoal[1])))
            var3.set(float("{:.2f}".format(xgoal[2])))
            var4.set(float("{:.2f}".format(xgoal[3])))
            var5.set(float("{:.2f}".format(xgoal[4])))
            var6.set(float("{:.2f}".format(xgoal[5])))

    # Create the main window
    root = tk.Tk()
    root.title("Leg Sliders")

    # Variables for sliders
    var1 = tk.DoubleVar()
    var2 = tk.DoubleVar()
    var3 = tk.DoubleVar()
    var4 = tk.DoubleVar()
    var5 = tk.DoubleVar()
    var6 = tk.DoubleVar()

    # Create the sliders
    slider1 = tk.Scale(root, from_=0, to=10, orient="horizontal", variable=var1, resolution=0.01, command=lambda _: update_values())
    slider1.pack(pady=5)

    slider2 = tk.Scale(root, from_=0, to=10, orient="horizontal", variable=var2, resolution=0.01, command=lambda _: update_values())
    slider2.pack(pady=5)

    slider3 = tk.Scale(root, from_=0, to=10, orient="horizontal", variable=var3, resolution=0.01, command=lambda _: update_values())
    slider3.pack(pady=5)

    slider4 = tk.Scale(root, from_=0, to=10, orient="horizontal", variable=var4, resolution=0.01, command=lambda _: update_values())
    slider4.pack(pady=5)

    slider5 = tk.Scale(root, from_=0, to=10, orient="horizontal", variable=var5, resolution=0.01, command=lambda _: update_values())
    slider5.pack(pady=5)

    slider6 = tk.Scale(root, from_=0, to=10, orient="horizontal", variable=var6, resolution=0.01, command=lambda _: update_values())
    slider6.pack(pady=5)

    # Set initial values
    var1.set(float("{:.2f}".format(xgoal[0])))
    var2.set(float("{:.2f}".format(xgoal[1])))
    var3.set(float("{:.2f}".format(xgoal[2])))
    var4.set(float("{:.2f}".format(xgoal[3])))
    var5.set(float("{:.2f}".format(xgoal[4])))
    var6.set(float("{:.2f}".format(xgoal[5])))

    # Labels to display the leg values
    label1 = tk.Label(root, text="Leg 1: "+str(var1.get()))
    label1.pack(pady=5)

    label2 = tk.Label(root, text="Leg 2: "+str(var2.get()))
    label2.pack(pady=5)

    label3 = tk.Label(root, text="Leg 3: "+str(var3.get()))
    label3.pack(pady=5)

    label4 = tk.Label(root, text="Leg 4: "+str(var4.get()))
    label4.pack(pady=5)

    label5 = tk.Label(root, text="Leg 5: "+str(var5.get()))
    label5.pack(pady=5)

    label6 = tk.Label(root, text="Leg 6: "+str(var6.get()))
    label6.pack(pady=5)


    # Run the GUI
    root.mainloop()
    #'''        