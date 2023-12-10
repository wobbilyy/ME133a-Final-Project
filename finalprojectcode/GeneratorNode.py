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
        ##### COMMENT OUT FOR FULL DEMO #####
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
        if x != None:
            self.marker.header.stamp  = now.to_msg()
            self.marker.pose.position.x = float(x[0])
            self.marker.pose.position.y = float(x[1])
            self.marker.pose.position.z = float(x[2] + 2.8)
            self.marker_publisher.publish(self.mark)
