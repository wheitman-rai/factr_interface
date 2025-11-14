import time
import numpy as np

import rclpy
from factr_interface.factr_teleop import Factr

# from gui import Gui

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class FACTRTeleopMujoco(Factr):
    """
    This class demonstrates the gravity compensation and null-space regulation function of the
    FACTR teleop leader arm. Communication between the leader arm and the follower Franka arm
    is not implemented in this example.
    """

    def __init__(self):
        super().__init__()
        # self.gui = Gui()
        # Store the latest torques for visualization
        self.latest_follower_torques = np.zeros(self.num_arm_joints)

    def set_up_communication(self):
        self.follower_state_sub = self.create_subscription(
            JointState, "/joint_states", self.follower_state_cb, 1
        )

        self.follower_external_torque_sub = self.create_subscription(
            JointState,
            "/franka_robot_state_broadcaster/external_joint_torques",
            self.follower_external_torque_cb,
            1,
        )

        self.follower_command_pub = self.create_publisher(
            JointState, "/hybrid_joint_impedance_controller/commands", 1
        )

        self.follower_gripper_command_pub = self.create_publisher(
            Float64MultiArray, "/robotiq_streaming_controller/commands", 1
        )

    def get_leader_gripper_feedback(self):
        pass

    def follower_state_cb(self, msg: JointState):
        self.latest_follower_state = msg

        expected_names = [f"fr3_joint{n}" for n in range(1, 8)]

        positions = []
        for expected_name in expected_names:
            for idx, actual_name in enumerate(msg.name):
                if expected_name == actual_name:
                    positions.append(msg.position[idx])

        self.latest_follower_positions = np.asarray(positions)

    def follower_external_torque_cb(self, msg: JointState):
        self.latest_follower_state = msg

        expected_names = [f"fr3_joint{n}" for n in range(1, 8)]

        efforts = []
        for expected_name in expected_names:
            for idx, actual_name in enumerate(msg.name):
                if expected_name == actual_name:
                    efforts.append(msg.effort[idx])

        self.latest_follower_torques = np.asarray(efforts)

    def gripper_feedback(
        self, leader_gripper_pos, leader_gripper_vel, gripper_feedback
    ):
        displacement = leader_gripper_pos + self.gripper_spring_displacement_offset
        # print(displacement)

        return -self.gripper_spring_constant * displacement

    def get_leader_arm_external_joint_torque(self):
        return self.latest_follower_torques

    def update_communication(self, leader_arm_pos: np.ndarray, leader_gripper_pos):
        """
        Transmit data from the leader (FACTR) to the follower (Franka).
        """

        # Publish arm commands
        command_msg = JointState()
        command_msg.name = [f"fr3_joint{n}" for n in range(1, 8)]
        command_msg.position = leader_arm_pos.tolist()
        self.follower_command_pub.publish(command_msg)

        # Publish gripper commands
        gripper_command_msg = Float64MultiArray()
        # TODO WSH: Properly map to [0.0,0.8]
        gripper_command_msg.data = [np.clip(leader_gripper_pos, 0.0, 0.8)]
        self.follower_gripper_command_pub.publish(gripper_command_msg)


def main(args=None):
    rclpy.init(args=args)
    factr_teleop_grav_comp = FACTRTeleopMujoco()

    try:
        while rclpy.ok():
            rclpy.spin(factr_teleop_grav_comp)
    except KeyboardInterrupt:
        factr_teleop_grav_comp.get_logger().info(
            "Keyboard interrupt received. Shutting down..."
        )
        factr_teleop_grav_comp.shut_down()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
