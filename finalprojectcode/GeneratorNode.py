'''GeneratorNode.py

   This creates a trajectory generator node

   from GeneratorNode import GeneratorNode
   generator = GeneratorNode(name, rate, TrajectoryClass)

      Initialize the node, under the specified name and rate.  This
      also requires a trajectory class which must implement:

         trajectory = TrajectoryClass(node)
         jointnames = trajectory.jointnames()
         (q, qdot)  = trajectory.evaluate(t, dt)

      where jointnames, q, qdot are all python lists of the joint's
      name, position, and velocity respectively.  The dt is the time
      since the last evaluation, to be used for integration.

      If trajectory.evaluate() return None, the node shuts down.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np
import tkinter as tk
from scipy.spatial.transform import Rotation

from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from rclpy.time                 import Duration
from geometry_msgs.msg          import Point, Vector3, Quaternion
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker
from visualization_msgs.msg     import MarkerArray

from asyncio            import Future
from rclpy.node         import Node
from sensor_msgs.msg    import JointState


#
#   Trajectory Generator Node Class
#
#   This inherits all the standard ROS node stuff, but adds
#     1) an update() method to be called regularly by an internal timer,
#     2) a spin() method, aware when a trajectory ends,
#     3) a shutdown() method to stop the timer.
#
#   Take the node name, the update frequency, and the trajectory class
#   as arguments.
#
class GeneratorNode(Node):
    # Initialization.
    def __init__(self, name, rate, Trajectory):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Set up a trajectory.
        self.trajectory = Trajectory(self)
        self.jointnames = self.trajectory.jointnames()

        # Add a publisher to send the joint commands.
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        ##### MARKER ARRAY TO RENDER TOP PLATFORM PLATE #####
        # Prepare the publisher (latching for new subscribers).
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', quality)

        # Create the sphere marker.
        self.radius = 5
        diam                         = float(1.0)
        self.marker                  = Marker()
        self.marker.header.frame_id  = "world"
        self.marker.header.stamp     = self.get_clock().now().to_msg()
        self.marker.action           = Marker.ADD
        self.marker.ns               = "point"
        self.marker.id               = 1
        self.marker.type             = Marker.MESH_RESOURCE
        self.marker.pose.position.x = float(1.0)
        self.marker.pose.position.y = float(1.0)
        self.marker.pose.position.z = float(1.0)
        self.marker.pose.orientation.x = 3.14159
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 0.0
        self.marker.scale            = Vector3(x = diam, y = diam, z = diam)
        self.marker.mesh_use_embedded_materials = True
        self.marker.mesh_resource = "package://finalprojectcode/meshes/Platform_Top.dae"

        # Create the marker array message.
        self.mark = MarkerArray()
        self.mark.markers.append(self.marker)
        ######

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # Create a future object to signal when the trajectory ends,
        # i.e. no longer returns useful data.
        self.future = Future()

        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now()+rclpy.time.Duration(seconds=self.dt)

        # Create a timer to keep calculating/sending commands.
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))
                             
        self.init_gui()

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()

    # Spin
    def spin(self):
        # Keep running (taking care of the timer callbacks and message
        # passing), until interrupted or the trajectory is complete
        # (as signaled by the future object).
        rclpy.spin_until_future_complete(self, self.future)

        # Report the reason for shutting down.
        if self.future.done():
            self.get_logger().info("Stopping: " + self.future.result())
        else:
            self.get_logger().info("Stopping: Interrupted")


    # Update - send a new joint command every time step.
    def update(self):
        # To avoid any time jitter enforce a constant time step and
        # integrate to get the current time.
        self.t += self.dt

        # Determine the corresponding ROS time (seconds since 1970).
        now = self.start + rclpy.time.Duration(seconds=self.t)

        # Compute the desired joint positions and velocities for this time.
        desired = self.trajectory.evaluate(self.t, self.dt)
        if desired is None:
            self.future.set_result("Trajectory has ended")
            return
        (q, qdot, x) = desired

        # Check the results.
        if not (isinstance(q, list) and isinstance(qdot, list)):
            self.get_logger().warn("(q) and (qdot) must be python lists!")
            return
        if not (len(q) == len(self.jointnames)):
            self.get_logger().warn("(q) must be same length as jointnames!")
            return
        if not (len(q) == len(self.jointnames)):
            self.get_logger().warn("(qdot) must be same length as (q)!")
            return
        if not (isinstance(q[0], float) and isinstance(qdot[0], float)):
            self.get_logger().warn("Flatten NumPy arrays before making lists!")
            return

        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = now.to_msg()      # Current time for ROS
        cmdmsg.name         = self.jointnames   # List of joint names
        cmdmsg.position     = q                 # List of joint positions
        cmdmsg.velocity     = qdot              # List of joint velocities
        self.pub.publish(cmdmsg)

        # publish platform top
        if x is not None:
            rot = Rotation.from_euler('xyz', x[3:], degrees=False).as_quat()
            self.marker.header.stamp  = now.to_msg()
            self.marker.pose.position.x = float(x[0])
            self.marker.pose.position.y = float(x[1])
            self.marker.pose.position.z = float(x[2] + 2.8)
            self.marker_publisher.publish(self.mark)
            self.marker.pose.orientation.x = 3.14159 + rot[0]
            self.marker.pose.orientation.y = -rot[1]
            self.marker.pose.orientation.z = -rot[2]
            self.marker.pose.orientation.w = -rot[3]
            
    #======================================================================
    # Stuff added by Ruth
    
    # Using K, we shall attempt to change the leg lengths with sliders and view the output.
    def update_values(self):
        self.trajectory.xgoal = [self.slider1.get(), self.slider2.get(), self.slider3.get(), self.slider4.get(), self.slider5.get(), self.slider6.get()]
        self.trajectory.qgoal  = self.trajectory.K.fkin(self.trajectory.xgoal)
        if (np.linalg.norm(np.array(self.trajectory.xgoal) - np.array(self.trajectory.K.invkin())) < 0.01):
            # Congrats, we actually converged. Update slider values.
            self.label1.config(text=f"Leg 1: {self.slider1.get():.2f}")
            self.label2.config(text=f"Leg 2: {self.slider2.get():.2f}")
            self.label3.config(text=f"Leg 3: {self.slider3.get():.2f}")
            self.label4.config(text=f"Leg 4: {self.slider4.get():.2f}")
            self.label5.config(text=f"Leg 5: {self.slider5.get():.2f}")
            self.label6.config(text=f"Leg 6: {self.slider6.get():.2f}")
            self.update()
        else:
            # Didn't converge. Set sliders to the last valid value.
            self.trajectory.xgoal = self.trajectory.K.invkin()
            self.var1.set(float("{:.2f}".format(self.trajectory.xgoal[0])))
            self.var2.set(float("{:.2f}".format(self.trajectory.xgoal[1])))
            self.var3.set(float("{:.2f}".format(self.trajectory.xgoal[2])))
            self.var4.set(float("{:.2f}".format(self.trajectory.xgoal[3])))
            self.var5.set(float("{:.2f}".format(self.trajectory.xgoal[4])))
            self.var6.set(float("{:.2f}".format(self.trajectory.xgoal[5])))
    
    def init_gui(self):
        
        # Create the main window
        self.root = tk.Tk()
        self.root.title("Leg Sliders")

        # Variables for sliders
        self.var1 = tk.DoubleVar()
        self.var2 = tk.DoubleVar()
        self.var3 = tk.DoubleVar()
        self.var4 = tk.DoubleVar()
        self.var5 = tk.DoubleVar()
        self.var6 = tk.DoubleVar()

        # Create the sliders
        self.slider1 = tk.Scale(self.root, from_=2.4, to=5, orient="horizontal", variable=self.var1, resolution=0.01, command=lambda _: self.update_values())
        self.slider1.pack(pady=5)

        self.slider2 = tk.Scale(self.root, from_=2.4, to=5, orient="horizontal", variable=self.var2, resolution=0.01, command=lambda _: self.update_values())
        self.slider2.pack(pady=5)

        self.slider3 = tk.Scale(self.root, from_=2.4, to=5, orient="horizontal", variable=self.var3, resolution=0.01, command=lambda _: self.update_values())
        self.slider3.pack(pady=5)

        self.slider4 = tk.Scale(self.root, from_=2.4, to=5, orient="horizontal", variable=self.var4, resolution=0.01, command=lambda _: self.update_values())
        self.slider4.pack(pady=5)

        self.slider5 = tk.Scale(self.root, from_=2.4, to=5, orient="horizontal", variable=self.var5, resolution=0.01, command=lambda _: self.update_values())
        self.slider5.pack(pady=5)

        self.slider6 = tk.Scale(self.root, from_=2.4, to=5, orient="horizontal", variable=self.var6, resolution=0.01, command=lambda _: self.update_values())
        self.slider6.pack(pady=5)

        # Set initial values
        self.var1.set(float("{:.2f}".format(self.trajectory.xgoal[0])))
        self.var2.set(float("{:.2f}".format(self.trajectory.xgoal[1])))
        self.var3.set(float("{:.2f}".format(self.trajectory.xgoal[2])))
        self.var4.set(float("{:.2f}".format(self.trajectory.xgoal[3])))
        self.var5.set(float("{:.2f}".format(self.trajectory.xgoal[4])))
        self.var6.set(float("{:.2f}".format(self.trajectory.xgoal[5])))

        # Labels to display the leg values
        self.label1 = tk.Label(self.root, text="Leg 1: "+str(self.var1.get()))
        self.label1.pack(pady=5)

        self.label2 = tk.Label(self.root, text="Leg 2: "+str(self.var2.get()))
        self.label2.pack(pady=5)

        self.label3 = tk.Label(self.root, text="Leg 3: "+str(self.var3.get()))
        self.label3.pack(pady=5)

        self.label4 = tk.Label(self.root, text="Leg 4: "+str(self.var4.get()))
        self.label4.pack(pady=5)

        self.label5 = tk.Label(self.root, text="Leg 5: "+str(self.var5.get()))
        self.label5.pack(pady=5)

        self.label6 = tk.Label(self.root, text="Leg 6: "+str(self.var6.get()))
        self.label6.pack(pady=5)
        
        self.update()

    def run_gui(self):
        self.root.mainloop()
