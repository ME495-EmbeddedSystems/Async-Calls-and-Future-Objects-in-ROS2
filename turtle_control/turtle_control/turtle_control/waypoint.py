import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum, auto
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Vector3
from math import pi
from random import uniform
from turtlesim.srv import Spawn, Kill
import time 

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    MOVING = auto(),
    STOPPED = auto()

class Waypoint(Node):

    def __init__(self):

        super().__init__("waypoint")
        self.state = State.STOPPED

        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="Frequency of messages sent"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)

    def timer_callback(self):

        if self.state == State.MOVING:
            self.get_logger().info("Issuing Command!")

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

def main(args=None):
    rclpy.init(args=args)
    node = Waypoint()

    # node.get_logger().info("Issuing Command!")

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
