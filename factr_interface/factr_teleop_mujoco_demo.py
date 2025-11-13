import time
import numpy as np

import rclpy
from factr_interface.factr_teleop import Factr
from gui import Gui

from sensor_msgs.msg import JointState


class FACTRTeleopMujoco(Factr):
    """
    This class demonstrates the gravity compensation and null-space regulation function of the
    FACTR teleop leader arm. Communication between the leader arm and the follower Franka arm
    is not implemented in this example.
    """

    def __init__(self):
        super().__init__()
        self.gui = Gui()
        # Store the latest torques for visualization
        self.latest_follower_torques = np.zeros(self.num_arm_joints)

    def set_up_communication(self):
        self.follower_state_sub = self.create_subscription(
            JointState, "/joint_states", self.follower_state_cb, 1
        )
        self.follower_command_pub = self.create_publisher(
            JointState, "/hybrid_joint_impedance_controller/commands", 1
        )

    def get_leader_gripper_feedback(self):
        pass

    def follower_state_cb(self, msg: JointState):
        self.latest_follower_state = msg

        expected_names = [f"fr3_joint{n}" for n in range(1, 8)]

        efforts = []
        positions = []
        for expected_name in expected_names:
            for idx, actual_name in enumerate(msg.name):
                if expected_name == actual_name:
                    efforts.append(msg.effort[idx])
                    positions.append(msg.position[idx])

        self.latest_follower_torques = np.asarray(efforts)
        self.latest_follower_positions = np.asarray(positions)

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
        Also update the Viser UI
        """

        # Update the GUI with torque data
        # print(f"Latest toruqes: {self.latest_leader_torques}")
        self.gui.update(leader_arm_pos, leader_gripper_pos, self.latest_leader_torques)

        command_msg = JointState()
        command_msg.name = [f"fr3_joint{n}" for n in range(1, 8)]

        # print(leader_arm_pos)

        # joint_error = np.abs(leader_arm_pos - self.latest_follower_positions)

        # if np.any(joint_error > 0.1):
        #     self.get_logger().warning(
        #         "Skipping command pub because joint error was too large"
        #     )
        #     self.get_logger().warning(f"{joint_error}")
        #     return

        command_msg.position = leader_arm_pos.tolist()

        # TODO WSH: Add gripper support

        self.follower_command_pub.publish(command_msg)


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
