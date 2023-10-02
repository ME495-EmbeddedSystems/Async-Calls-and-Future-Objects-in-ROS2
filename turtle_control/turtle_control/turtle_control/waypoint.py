import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum, auto
from crazy_turtle_interfaces.srv import Switch
from geometry_msgs.msg import Twist, Vector3
from math import pi
from random import uniform
from turtlesim.srv import Spawn, Kill
import time 

class Waypoint(Node):

    def __init__(self):

        super().__init__("waypoint")

        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="Frequency of messages sent"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

    def timer_callback(self):

        self.get_logger().debug("Issuing Command!")

def main(args=None):
    rclpy.init(args=args)
    node = Waypoint()

    # node.get_logger().info("Issuing Command!")

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
