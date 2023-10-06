import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum, auto
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoints
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

class Waypoint(Node):

    def __init__(self):

        super().__init__("waypoint")
        self.state = State.STOPPED

        self.callback_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="Frequency of messages sent"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value

        self.originalX = 0
        self.originalY = 0

        self.newX = 0
        self.newY = 0

        self.drawing = 0
        self.cross_length = 1
        self.velocity = 10.0

        self.waypoints = np.zeros([2,100])
        self.num_waypoints = 0
        self.ctr = 0
        self.nsteps = 0

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.pose_subscriber = self.create_subscription(Pose,'/turtle1/pose', self.update_pose, 10)
 
        self.pose = Pose()
        
        self.frequency = 1000.0 # Debugging tool
        # self.get_logger().info(f"Frequency: {self.frequency}")
        self.timer = self.create_timer(1/self.frequency, self.timer_callback, callback_group=self.callback_group)

        self.teleport_future = None
        self.setpen_future = None
        self.skip = 0

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

        # self.get_logger().info(f"{self.pose.x}, {self.pose.y}")
        
        if self.state == State.MOVING:
            self.get_logger().info("Issuing Command!")

            # twist = turtle_twist(self.velocity, 0)
                
            # self.pub.publish(twist)

        elif self.state == State.APPARATING:
            # self.get_logger().info("turtle1 is about to apparate!")

            # Move to top left corner or Reset turtle
            if self.drawing == 1:
                self.get_logger().info(f"{self.drawing} Waiting for Pen")
                
                self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=True))

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


            # Draw first line
            elif self.drawing == 2:

                self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=False))

                    
                self.newX = self.waypoints[0, self.ctr - 1] + self.cross_length
                self.newY = self.waypoints[1, self.ctr - 1] - self.cross_length
                
                self.drawing = 3

                self.get_logger().info(f"{self.drawing} Waiting for Pen")

            # Move to top right corner
            elif self.drawing == 3:

                self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=True))

                    
                self.newX = self.waypoints[0, self.ctr - 1] + self.cross_length
                self.newY = self.waypoints[1, self.ctr - 1] + self.cross_length
                
                self.drawing = 4

                self.get_logger().info(f"{self.drawing} Waiting for Pen")

            # Draw last line
            elif self.drawing == 4:

                self.setpen_future = self.setpen.call_async(SetPen.Request(r=200, g=200, b=200, width=4, off=False))
                    
                self.newX = self.waypoints[0, self.ctr - 1] - self.cross_length
                self.newY = self.waypoints[1, self.ctr - 1] - self.cross_length
                
                self.drawing = 1

                self.get_logger().info(f"{self.drawing} Waiting for Pen")
                        
            self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = self.newX, y = self.newY, theta = 0.0, name="turtle1"))

            self.get_logger().info("Didn't teleport")   

                
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

        elif self.state == State.MOVING:

            self.state = State.STOPPED
            self.get_logger().info("Stopping")

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

        # Remember original position
        self.originalX = self.pose.x # for now
        self.originalY = self.pose.y # For now

        response.dist = 0

        for i in range(self.num_waypoints - 1):

            response.dist += np.linalg.norm(self.waypoints[:,i] - self.waypoints[:,i+1])

        response.dist += np.linalg.norm(self.waypoints[:,0] - [self.originalX, self.originalY])
        response.dist += np.linalg.norm(self.waypoints[:,self.num_waypoints - 1] - [self.originalX, self.originalY])

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
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)



def main(args=None):
    rclpy.init(args=args)
    node = Waypoint()

    node.get_logger().info("Starting Trial")

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
