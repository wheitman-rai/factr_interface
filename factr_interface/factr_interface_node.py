# -----------------------------------------------------------------------------
# Description: Bridges data from Dynamixel motors and robot arms (Franka, Kinova)
# Author: Will Heitman
# (c) 2025 RAI Institute
# -----------------------------------------------------------------------------

import numpy as np
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node, ParameterDescriptor, ParameterType


# Messages
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class FactrInterfaceNode(Node):
    """
    Bridges data from Dynamixel motors and robot arms (Franka, Kinova)
    """

    def __init__(self):
        super().__init__("factr_interface")

        self.get_logger().info(f"Hello from the FACTR Interface!")




def main(args=None):
    rclpy.init(args=args)

    node = FactrInterfaceNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()