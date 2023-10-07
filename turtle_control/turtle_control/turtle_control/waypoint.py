import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum, auto
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoints
from turtle_interfaces.msg import ErrorMetric
from geometry_msgs.msg import Twist, Vector3
from math import pi
from random import uniform
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import time 
import math
import numpy as np

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    MOVING = auto(),
    STOPPED = auto(),
    APPARATING = auto()

def turtle_twist(xdot, omega):
    """ Create a twist suitable for a turtle

        Args:
           xdot (float) : the forward velocity
           omega (float) : the angular velocity

        Returns:
           Twist - a 2D twist object corresponding to linear/angular velocity
    """
    return Twist(linear = Vector3(x = xdot, y = 0.0, z = 0.0),
                  angular = Vector3(x = 0.0, y = 0.0, z = omega))

def loop_info(complete_loops, actual_distance, error):

    return ErrorMetric(complete_loops=complete_loops, actual_distance=actual_distance, error=error)

class Waypoint(Node):

    def __init__(self):

        super().__init__("waypoint")
        self.state = State.STOPPED

        self.callback_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="Frequency of messages sent"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value

        self.declare_parameter("tolerance", 0.05,
                               ParameterDescriptor(description="Distance tolerance of motion controller"))
        self.dtol = self.get_parameter("tolerance").get_parameter_value().double_value

        self.originalX = 0
        self.originalY = 0

        self.newX = 0
        self.newY = 0

        self.drawing = 0
        self.cross_length = 0.1
        # self.cross_length = 2
        self.following = 0
        self.velocity = 10.0
        self.ang_vel = 0.0
        self.fctr = 1
        # self.dtol = 0.05
        self.angtol = 0.01

        self.waypoints = np.zeros([2,100])
        self.num_waypoints = 0
        self.ctr = 0
        self.nsteps = 0

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.loop_pub = self.create_publisher(ErrorMetric, "/loop_metrics", 10)

        self.pose_subscriber = self.create_subscription(Pose,'/turtle1/pose', self.update_pose, 10)
 
        self.pose = Pose()

        self.prevX = 0
        self.prevY = 0
        
        self.loopctr = 0
        self.actual_distance = 0.0
        self.error = 0.0

        self.errormsg = 0

        # self.frequency = 2000.0 # Debugging tool
        # self.get_logger().info(f"Frequency: {self.frequency}")
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

        self.teleport_future = None
        self.setpen_future = None
        self.skip = False

        self.kill_future = 0 
        self.kill      = self.create_client(Kill, "kill")
        self.spawn     = self.create_client(Spawn,"spawn")
        self.reset     = self.create_client(Empty, "reset")
        self.teleport = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute", callback_group=self.callback_group)
        self.setpen = self.create_client(SetPen, "turtle1/set_pen", callback_group=self.callback_group)

        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)
        # self.toggle = self.create_service(Empty, "toggle", self.toggle_callback, self.callback_group)

        self.load = self.create_service(Waypoints, "load", self.load_callback)
        # self.load = self.create_service(Waypoints, "load", self.load_callback, self.callback_group)

    def timer_callback(self):

        # self.get_logger().info(f"h {self.pose.x}, {self.pose.y}, {self.pose.theta}")
        
        if self.state == State.MOVING:

            if self.num_waypoints == 0:

                self.get_logger().error("No waypoints loaded. Load them with the \"load\" service")
                self.state = State.STOPPED

            else:

                target_angle = math.atan2(self.waypoints[1, self.fctr] - self.pose.y, self.waypoints[0, self.fctr] - self.pose.x)

                anglediff = target_angle - self.pose.theta

                anglediff = math.atan2(math.sin(anglediff), math.cos(anglediff))

                if anglediff > self.angtol:

                    self.ang_vel = 1.0

                elif anglediff < -self.angtol:

                    self.ang_vel = -1.0

                else:

                    self.ang_vel = 0.0
                    self.following = 2

                if self.following == 1:

                    self.velocity = 0.0
                    
                if self.following == 2:

                    self.velocity = 5.0

                    posdiff = np.linalg.norm(self.waypoints[:,self.fctr] - [self.pose.x, self.pose.y])

                    # self.get_logger().info(f"{self.waypoints[:,self.fctr], [self.pose.x, self.pose.y]}")

                    if posdiff < self.dtol and posdiff > -self.dtol:

                        self.velocity = 0.0

                        self.following = 1

                        # self.get_logger().info(f"fctr: {self.fctr}")

                        if self.fctr == 0:

                            self.loopctr = self.loopctr + 1

                            self.error = self.actual_distance - self.waypoint_dist

                            self.errormsg = loop_info(self.loopctr, self.actual_distance, self.error)

                            self.loop_pub.publish(self.errormsg)

                            self.actual_distance = 0

                        self.fctr = self.fctr + 1

                        if self.fctr == self.num_waypoints:

                            self.fctr = 0

                            # self.actual_distance = 0

                twist = turtle_twist(self.velocity, self.ang_vel)
                # self.get_logger().info("Issuing Command!")
                self.pub.publish(twist)

                self.actual_distance = self.actual_distance + np.linalg.norm([self.prevX - self.pose.x, self.prevY - self.pose.y])

                self.prevX = self.pose.x
                self.prevY = self.pose.y

                self.errormsg = loop_info(self.loopctr, self.actual_distance, self.error)
                # self.get_logger().info(f"{self.errormsg}")


        elif self.state == State.APPARATING:
            # self.get_logger().info("turtle1 is about to apparate!")

            # Move to top left corner or Reset turtle
            if self.drawing == 1 and not self.skip:
                
                if self.setpen_future is None:

                    self.get_logger().info(f"{self.setpen_future, self.drawing} Calling SetPen")
                    self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=True))

                if self.setpen_future.done():

                    self.get_logger().info(f"Pen worked")

                    if self.ctr >= self.num_waypoints:

                        self.newX = self.originalX
                        self.newY = self.originalY

                        self.drawing = 0
                        self.state = State.STOPPED
                        self.ctr = 0

                    else: 
                
                        self.ctr = self.ctr + 1

                        self.newX = self.waypoints[0, self.ctr - 1] - self.cross_length
                        self.newY = self.waypoints[1, self.ctr - 1] + self.cross_length
                    
                        self.drawing = 2

                else:
                
                    self.get_logger().info(f"{self.drawing} Waiting for Pen")

            # Draw first line
            elif self.drawing == 2 and not self.skip:

                if self.setpen_future is None:

                    self.get_logger().info(f"{self.setpen_future, self.drawing} Calling SetPen")
                    self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=False))

                if self.setpen_future.done():

                    self.get_logger().info("Pen worked")
                    
                    self.newX = self.waypoints[0, self.ctr - 1] + self.cross_length
                    self.newY = self.waypoints[1, self.ctr - 1] - self.cross_length
                
                    self.drawing = 3

                else:

                    self.get_logger().info(f"{self.drawing} Waiting for Pen")

            # Move to top right corner
            elif self.drawing == 3 and not self.skip:

                if self.setpen_future is None:

                    self.get_logger().info(f"{self.setpen_future, self.drawing} Calling SetPen")
                    self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=True))

                if self.setpen_future.done():

                    self.get_logger().info("Pen worked")
                    
                    self.newX = self.waypoints[0, self.ctr - 1] + self.cross_length
                    self.newY = self.waypoints[1, self.ctr - 1] + self.cross_length
                
                    self.drawing = 4

                else:

                    self.get_logger().info(f"{self.drawing} Waiting for Pen")

            # Draw last line
            elif self.drawing == 4 and not self.skip:

                if self.setpen_future is None:

                    self.get_logger().info(f"{self.setpen_future, self.drawing} Calling SetPen")
                    self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=False))

                if self.setpen_future.done():

                    self.get_logger().info("Pen worked")
                    
                    self.newX = self.waypoints[0, self.ctr - 1] - self.cross_length
                    self.newY = self.waypoints[1, self.ctr - 1] - self.cross_length
                
                    self.drawing = 1

                else:

                    self.get_logger().info(f"{self.drawing} Waiting for Pen")
                        
            if self.setpen_future.done():

                if self.teleport_future is None:

                    self.get_logger().info("Calling Teleport")
                    self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = self.newX, y = self.newY, theta = 0.0, name="turtle1"))

                if self.teleport_future.done():

                    self.get_logger().info("Teleporting!")
                    self.setpen_future = None
                    self.teleport_future = None
                    self.skip = False

                else:

                    self.get_logger().info("Waiting for Teleport")
                    self.skip = True

            else:

                self.get_logger().info("Didn't call teleport")

            # self.get_logger().info(f" {self.drawing} turtle1 apparated!")
                

    def toggle_callback(self, request, response):
        """ Callback function for the toggleservice

         Args:
          request (SwitchRequest): the mixer field contains
             x, y, linear and angular velocity components
             that are used to determine the new turtle location

          response (SwitchResponse): the response object

        Returns:
           A SwitchResponse, containing the new x and y position
        """

        if self.state == State.STOPPED:

            self.state = State.MOVING

            self.following = 1
            self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=False))
            # self.fctr = 1

        elif self.state == State.MOVING:

            self.state = State.STOPPED
            self.get_logger().info("Stopping")
            twist = turtle_twist(0.0, 0.0)
            self.pub.publish(twist)
            # self.fctr = 1

        return response

    def load_callback(self, request, response):

        self.get_logger().info("Loading Waypoints")
        # self.kill_future = self.kill.call_async(Kill.Request(name="turtle1"))

        self.state = State.APPARATING
        self.drawing = 1

        # Load array of waypoints into node
        self.num_waypoints = len(request.x_arr)
        self.waypoints[0,0:self.num_waypoints] = request.x_arr[:]
        self.waypoints[1,0:self.num_waypoints] = request.y_arr[:]

        self.loopctr = 0
        self.actual_distance = 0.0
        self.error = 0.0
        self.errormsg = loop_info(self.loopctr, self.actual_distance, self.error)
        self.loop_pub.publish(self.errormsg)

        # Start point for following waypoints
        self.originalX = self.waypoints[0,0] # for now
        self.originalY = self.waypoints[1,0] # For now

        self.prevX = self.originalX
        self.prevY = self.originalY

        self.get_logger().info(f"Initial position: {self.originalX, self.originalY}")

        response.dist = 0

        for i in range(self.num_waypoints - 1):

            response.dist += np.linalg.norm(self.waypoints[:,i] - self.waypoints[:,i+1])

        response.dist += np.linalg.norm(self.waypoints[:,0] - [self.originalX, self.originalY])
        response.dist += np.linalg.norm(self.waypoints[:,self.num_waypoints - 1] - [self.originalX, self.originalY])

        self.waypoint_dist = response.dist

        # self.get_logger().info(f"DIST: {self.dist}")

        # Select first waypoint
        # self.newX = self.waypoints[0, 0] - self.cross_length
        # self.newY = self.waypoints[1, 0] - self.cross_length

        # Lift pen up
        # self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=False))

        self.get_logger().info(f"{self.waypoints[:,0:self.num_waypoints]}")
        
        return response

    def update_pose(self, data):

        self.pose = data

        # self.get_logger().error(f"{data}")



def main(args=None):
    rclpy.init(args=args)
    node = Waypoint()

    node.get_logger().info("Starting Trial")

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
