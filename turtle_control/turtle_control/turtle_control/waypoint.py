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
        Determines what the main timer function should be doing on each iteration.
    """
    MOVING = auto(),
    STOPPED = auto(),
    APPARATING = auto()

def turtle_twist(xdot, omega):
    """ Create a twist suitable for a turtle.

        Args:
           xdot (float) : the forward velocity
           omega (float) : the angular velocity

        Returns:
           Twist - a 2D twist object corresponding to linear/angular velocity
    """
    return Twist(linear = Vector3(x = xdot, y = 0.0, z = 0.0),
                  angular = Vector3(x = 0.0, y = 0.0, z = omega))

def loop_info(complete_loops, actual_distance, error):
    """ Create a formatted object for ErrorMetric message type.

        Args:
           complete_loops (int) : number of loops completed by the turtle
           actual_distance (float) : distance traveled by turtle measured by summing pose feedback
           error (float) : difference between actual distance traveled by turtle and minmum distances between waypoints of the loop

        Returns:
           ErrorMessage - an object with 3 values
    """

    return ErrorMetric(complete_loops=complete_loops, actual_distance=actual_distance, error=error)

class Waypoint(Node):

    """ Create a Class for ros2 node for controlling a turtle.

    Members:
        __init__ : Instantiate Class objects
        timer_callback : Callback for timer
        toggle_callabck : Callback for /toggle service
        load_callback : Callback for /load service
        update_pose : Callback for /turtle1/pose subscriber
    """

    def __init__(self):

        """ Instantiate Class Objects including publishers, subscribers, services, and clients.
        """

        # Initially the turtle is statioanry
        super().__init__("waypoint")
        self.state = State.STOPPED

        # Callback group for set_pen and teleport services
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Parametrize timer frequency
        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="Frequency of timer service"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value

        # Parametrize distance tolerance
        self.declare_parameter("tolerance", 0.05,
                               ParameterDescriptor(description="Distance tolerance of motion controller"))
        self.dtol = self.get_parameter("tolerance").get_parameter_value().double_value

        # Original position of turtle for resetting
        self.originalX = 0
        self.originalY = 0

        # Commanded position to turtle
        self.newX = 0
        self.newY = 0

        # Previous feedback position for distance calculation
        self.prevX = 0
        self.prevY = 0

        # Initialize drawing variables
        self.drawing = 0 # Sub-state for drawing crosses
        self.cross_length = 0.1 # Half of length of one cross in meters
        self.ctr = 0  # Tracker for drawing waypoints
        self.teleport_future = None # Future object for asynchronous teleport call
        self.setpen_future = None # Future object for asynchronous setpen call
        self.skip = False # Tracking object

        # Initialize motion control variables
        self.following = 1 # Sub-state for motion-controller
        self.velocity = 10.0 # Forward velocity
        self.ang_vel = 0.0 # Angular velocity
        self.fctr = 1 # Loop segment tracker
        self.angtol = 0.01 # Angular tolerance

        # Initialize waypoint variables
        self.waypoints = np.zeros([2,100]) # Waypoints list
        self.num_waypoints = 0 # Number of waypoints

        # Initialize Loop metrics
        self.loopctr = 0 # Number of completed loops
        self.actual_distance = 0.0 # Distance traveled by turtle
        self.error = 0.0 # Error in distance traveled
        self.errormsg = 0 # ErrorMetric Object

        # Publishers

        # Create Publisher for /cmd_vel topic
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Create Publisher for /loop_metrics topic
        self.loop_pub = self.create_publisher(ErrorMetric, "/loop_metrics", 10)

        # Subscribers

        # Create subscriber for pose feedback
        self.pose_subscriber = self.create_subscription(Pose,'/turtle1/pose', self.update_pose, 10)
        self.pose = Pose() # Feedback pose

        # Clients
        
        # Create reset client
        self.reset     = self.create_client(Empty, "reset")

        # Create teleport client
        self.teleport = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute", callback_group=self.callback_group)
        
        # Create set_pen client
        self.setpen = self.create_client(SetPen, "turtle1/set_pen", callback_group=self.callback_group)

        # Services
        
        # Create timer service
        # self.frequency = 2000.0 # Debugging tool
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

        # Create /toggle service
        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)

        # Create /load service
        self.load = self.create_service(Waypoints, "load", self.load_callback)

    def timer_callback(self):

        """ Timer callback to be called at a particular frequency.
            Determines behaviour based on state of the system.
        """

        # Behaviour when turtle is moving
        if self.state == State.MOVING:

            # Stop and reset motion controller when no waypoints are loaded
            if self.num_waypoints == 0:

                self.get_logger().error("No waypoints loaded. Load them with the \"load\" service")
                self.state = State.STOPPED

            else:

                # Make turtle face target waypoint
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

                # Keep turtle stationary until it faces the right direction
                if self.following == 1:

                    self.velocity = 0.0
                    
                # When turtle faces the right direction:
                if self.following == 2:

                    # Make turtle move forward until it is close enough to the target wayoint
                    self.velocity = 5.0 
                    posdiff = np.linalg.norm(self.waypoints[:,self.fctr] - [self.pose.x, self.pose.y])
                    if posdiff < self.dtol and posdiff > -self.dtol:

                        self.velocity = 0.0

                        # When loop is closed, calculate loop metrics and publish to /loop_metrics topic
                        if self.fctr == 0:

                            self.loopctr = self.loopctr + 1
                            self.error = self.actual_distance - self.waypoint_dist
                            self.errormsg = loop_info(self.loopctr, self.actual_distance, self.error)

                            self.loop_pub.publish(self.errormsg)

                            self.actual_distance = 0

                        # Change target waypoint
                        self.fctr = self.fctr + 1

                        # Set first waypoint to be the target when turtle is on the last waypoint
                        if self.fctr == self.num_waypoints:

                            self.fctr = 0
                        
                        self.following = 1 # To remain stationary until it faces the new right direction

                # Publish twist
                twist = turtle_twist(self.velocity, self.ang_vel)
                self.pub.publish(twist)

                # Update distance measurement
                self.actual_distance = self.actual_distance + np.linalg.norm([self.prevX - self.pose.x, self.prevY - self.pose.y])
                self.prevX = self.pose.x
                self.prevY = self.pose.y

        # Behaviour when turtle is teleporting to draw crosses on waypoints
        elif self.state == State.APPARATING:

            # Move to top left corner or Reset turtle
            if self.drawing == 1 and not self.skip:
                
                # Call set_pen service if it hasn't been called 
                if self.setpen_future is None:

                    self.get_logger().info(f"{self.setpen_future, self.drawing} Calling SetPen")
                    self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=True))

                # Assign point to teleport to after set_pen responds
                if self.setpen_future.done():

                    self.get_logger().info(f"Pen worked")

                    # Stop when waypoints are all drawn
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

                # Pass if not yet responded
                else:
                
                    self.get_logger().info(f"{self.drawing} Waiting for Pen")

            # Draw first line
            elif self.drawing == 2 and not self.skip:

                # Call set_pen service if it hasn't been called 
                if self.setpen_future is None:

                    self.get_logger().info(f"{self.setpen_future, self.drawing} Calling SetPen")
                    self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=False))

                # Assign point to teleport to after set_pen responds
                if self.setpen_future.done():

                    self.get_logger().info("Pen worked")
                    
                    self.newX = self.waypoints[0, self.ctr - 1] + self.cross_length
                    self.newY = self.waypoints[1, self.ctr - 1] - self.cross_length
                
                    self.drawing = 3

                # Pass if not yet responded
                else:

                    self.get_logger().info(f"{self.drawing} Waiting for Pen")

            # Move to top right corner
            elif self.drawing == 3 and not self.skip:

                # Call set_pen service if it hasn't been called 
                if self.setpen_future is None:

                    self.get_logger().info(f"{self.setpen_future, self.drawing} Calling SetPen")
                    self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=True))

                # Assign point to teleport to after set_pen responds
                if self.setpen_future.done():

                    self.get_logger().info("Pen worked")
                    
                    self.newX = self.waypoints[0, self.ctr - 1] + self.cross_length
                    self.newY = self.waypoints[1, self.ctr - 1] + self.cross_length
                
                    self.drawing = 4

                # Pass if not yet responded
                else:

                    self.get_logger().info(f"{self.drawing} Waiting for Pen")

            # Draw last line
            elif self.drawing == 4 and not self.skip:

                # Call set_pen service if it hasn't been called 
                if self.setpen_future is None:

                    self.get_logger().info(f"{self.setpen_future, self.drawing} Calling SetPen")
                    self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=False))

                # Assign point to teleport to after set_pen responds
                if self.setpen_future.done():

                    self.get_logger().info("Pen worked")
                    
                    self.newX = self.waypoints[0, self.ctr - 1] - self.cross_length
                    self.newY = self.waypoints[1, self.ctr - 1] - self.cross_length
                
                    self.drawing = 1

                # Pass if not yet responded
                else:

                    self.get_logger().info(f"{self.drawing} Waiting for Pen")
                        
            # Call teleport service after set_pen responds
            if self.setpen_future.done():

                # Call teleport service if it hasn't been called
                if self.teleport_future is None:

                    self.get_logger().info("Calling Teleport")
                    self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = self.newX, y = self.newY, theta = 0.0, name="turtle1"))

                # Reset parameters after teleport responds
                if self.teleport_future.done():

                    self.get_logger().info("Teleporting!")
                    self.setpen_future = None
                    self.teleport_future = None
                    self.skip = False

                # Pass if not yet responded and skip all pen calls
                else:

                    self.get_logger().info("Waiting for Teleport")
                    self.skip = True

            # Don't call teleport if set_pen didn't respond
            else:

                self.get_logger().info("Didn't call teleport")                

    def toggle_callback(self, request, response):
        """ Callback function for the toggle service

            Args:
                request (Empty_Request): empty request passed to indicate toggling the state of the system

                response (Empty_Response): empty response received
        """

        # Toggle state from stopped to moving
        if self.state == State.STOPPED:

            self.state = State.MOVING

            self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=False)) # Use pen to track motion

        # Toggle state from moving to stopped
        elif self.state == State.MOVING:

            self.state = State.STOPPED

            self.get_logger().info("Stopping")

            # Reset turtle twist to zero
            twist = turtle_twist(0.0, 0.0)
            self.pub.publish(twist)

        return response

    def load_callback(self, request, response):

        """ Callback function for the load service.
            Stores waypoints within the node and initiates drawing of waypoints as little crosses

            Args:
                request (Waypoints_Request): Contains the ordered list of X-coordinates (x_arr) and list of Y-coordinates of waypoints (y_arr) to be drawn

                response (Waypoints_Response): Displacement required to traverse a closed loop formed by the waypoints

            Returns:
                A float measuring the displacement in a loop formed by the waypoints
        """
        
        self.get_logger().info("Loading Waypoints")

        self.state = State.APPARATING # Initiate teleporting state for drawing 
        self.drawing = 1 # Initiate substate for drawing each cross

        # Load array of waypoints into node
        self.num_waypoints = len(request.x_arr)
        self.waypoints[0,0:self.num_waypoints] = request.x_arr[:]
        self.waypoints[1,0:self.num_waypoints] = request.y_arr[:]

        # Reset loop metrics
        self.loopctr = 0
        self.actual_distance = 0.0
        self.error = 0.0
        self.errormsg = loop_info(self.loopctr, self.actual_distance, self.error)
        self.loop_pub.publish(self.errormsg)

        # Record position to finish drawing at
        self.originalX = self.waypoints[0,0] # for now
        self.originalY = self.waypoints[1,0] # For now

        # Reset position feedback distance measurement
        self.prevX = self.originalX
        self.prevY = self.originalY

        # Calculate displacement in loop of waypoints
        response.dist = 0
        for i in range(self.num_waypoints - 1):

            response.dist += np.linalg.norm(self.waypoints[:,i] - self.waypoints[:,i+1])

        response.dist += np.linalg.norm(self.waypoints[:,0] - [self.originalX, self.originalY])
        response.dist += np.linalg.norm(self.waypoints[:,self.num_waypoints - 1] - [self.originalX, self.originalY])

        self.waypoint_dist = response.dist # Keep copy within the node for error calcualtion
        
        # Return response object
        return response

    def update_pose(self, data):
        """ Stores current pose feedback data from /turtle1/pose

            Args: 
                data : object containing x, y, and theta of turtle's pose
        """
        self.pose = data

def main(args=None):
    """ Main function to read arguments, create node object and indicate successful node creation
    """

    rclpy.init(args=args)
    node = Waypoint()

    node.get_logger().info("Waypoint node created")

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
