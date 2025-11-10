import time
import numpy as np

import rclpy
from factr_interface.factr_teleop import Factr
from gui import Gui


class FACTRTeleopGravComp(Factr):
    """
    This class demonstrates the gravity compensation and null-space regulation function of the
    FACTR teleop leader arm. Communication between the leader arm and the follower Franka arm
    is not implemented in this example.
    """

    def __init__(self):
        super().__init__()
        self.gui = Gui()
        # Store the latest torques for visualization
        self.latest_torques = np.zeros(self.num_arm_joints)

    def set_up_communication(self):
        pass

    def get_leader_gripper_feedback(self):
        pass

    def gripper_feedback(
        self, leader_gripper_pos, leader_gripper_vel, gripper_feedback
    ):
        displacement = leader_gripper_pos + self.gripper_spring_displacement_offset
        print(displacement)

        return -self.gripper_spring_constant * displacement

    def get_leader_arm_external_joint_torque(self):
        pass

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        """
        Transmit data from the leader (FACTR) to the follower (Franka).
        Also update the Viser UI
        """

        # Update the GUI with torque data
        self.gui.update(leader_arm_pos, leader_gripper_pos, self.latest_torques)

        pass


def main(args=None):
    rclpy.init(args=args)
    factr_teleop_grav_comp = FACTRTeleopGravComp()

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
