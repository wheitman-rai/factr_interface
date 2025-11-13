import os
import time
import yaml
import subprocess
import numpy as np
import pinocchio as pin
from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node
from factr_interface.dynamixel.driver import DynamixelDriver
from dynamixel_sdk.robotis_def import COMM_SUCCESS


def find_ttyusb(port_name):
    """
    This function is used to locate the underlying ttyUSB device.
    """
    base_path = "/dev/serial/by-id/"
    full_path = os.path.join(base_path, port_name)
    if not os.path.exists(full_path):
        raise Exception(f"Port '{port_name}' does not exist in {base_path}.")
    try:
        resolved_path = os.readlink(full_path)
        actual_device = os.path.basename(resolved_path)
        if actual_device.startswith("ttyUSB"):
            return actual_device
        else:
            raise Exception(
                f"The port '{port_name}' does not correspond to a ttyUSB device. It links to {resolved_path}."
            )
    except Exception as e:
        raise Exception(f"Unable to resolve the symbolic link for '{port_name}'. {e}")


class Factr(Node, ABC):
    """
    Base class for implementing the FACTR low-cost force-feedback teleoperation system
    for a follower arm.

    This class implements the control loop for the leader teleoperation arm, including
    features such as gravity compensation, null-space regulation, friction compensation,
    and force-feedback.

    Note that this class should be used as a parent class, where the defined abstract
    methods must be implemented by subclasses for handling communication between the
    leader and follower arms, as well as force-feedback for the leader gripper.
    """

    def __init__(self):
        super().__init__("factr_teleop")

        # TODO WSH: PARAMETERIZE THIS PATH
        with open(
            "/home/wheitman/wheitman_ws/src/factr_interface/factr_interface/configs/config.yaml",
            "r",
        ) as config_file:
            self.config = yaml.safe_load(config_file)

        self.name = self.config["name"]
        self.dt = 1 / self.config["controller"]["frequency"]

        self.latest_follower_positions = None
        self.latest_leader_torques = np.zeros(7)

        self._prepare_dynamixel()
        self._prepare_inverse_dynamics()

        # leader arm parameters
        self.num_arm_joints = self.config["arm_teleop"]["num_arm_joints"]
        self.safety_margin = self.config["arm_teleop"]["arm_joint_limits_safety_margin"]
        self.arm_joint_limits_max = (
            np.array(self.config["arm_teleop"]["arm_joint_limits_max"])
            - self.safety_margin
        )
        self.arm_joint_limits_min = (
            np.array(self.config["arm_teleop"]["arm_joint_limits_min"])
            + self.safety_margin
        )
        self.calibration_joint_pos = np.array(
            self.config["arm_teleop"]["initialization"]["calibration_joint_pos"]
        )
        self.initial_match_joint_pos = np.array(
            self.config["arm_teleop"]["initialization"]["initial_match_joint_pos"]
        )

        self.global_torque_limit = self.config["controller"]["global_torque_limit"]

        assert (
            self.num_arm_joints
            == len(self.arm_joint_limits_max)
            == len(self.arm_joint_limits_min)
        ), "num_arm_joints and the length of arm joint limits must be the same"
        assert (
            self.num_arm_joints
            == len(self.calibration_joint_pos)
            == len(self.initial_match_joint_pos)
        ), "num_arm_joints and the length of calibration_joint_pos and initial_match_joint_pos must be the same"

        # leader gripper parameters
        self.gripper_limit_min = 0.0
        self.gripper_limit_max = self.config["gripper_teleop"]["actuation_range"]
        self.gripper_pos_prev = 0.0
        self.gripper_pos = 0.0

        # gravity comp
        self.enable_gravity_comp = self.config["controller"]["gravity_comp"]["enable"]
        self.gravity_comp_modifier = self.config["controller"]["gravity_comp"]["gain"]
        self.tau_g = np.zeros(self.num_arm_joints)
        # friction comp
        self.stiction_comp_enable_speed = self.config["controller"][
            "static_friction_comp"
        ]["enable_speed"]
        self.stiction_comp_gain = self.config["controller"]["static_friction_comp"][
            "gain"
        ]
        self.stiction_dither_flag = np.ones((self.num_arm_joints), dtype=bool)
        # joint limit barrier:
        self.joint_limit_kp = self.config["controller"]["joint_limit_barrier"]["kp"]
        self.joint_limit_kd = self.config["controller"]["joint_limit_barrier"]["kd"]
        # null space regulation
        self.null_space_joint_target = np.array(
            self.config["controller"]["null_space_regulation"][
                "null_space_joint_target"
            ]
        )
        self.null_space_kp = self.config["controller"]["null_space_regulation"]["kp"]
        self.null_space_kd = self.config["controller"]["null_space_regulation"]["kd"]
        # torque feedback
        self.enable_torque_feedback = self.config["controller"]["torque_feedback"][
            "enable"
        ]
        self.torque_feedback_gain = self.config["controller"]["torque_feedback"]["gain"]
        self.torque_feedback_motor_scalar = self.config["controller"][
            "torque_feedback"
        ]["motor_scalar"]
        self.torque_feedback_damping = self.config["controller"]["torque_feedback"][
            "damping"
        ]
        # gripper feedback
        self.enable_gripper_feedback = self.config["controller"]["gripper_feedback"][
            "enable"
        ]

        self.gripper_spring_constant = self.config["controller"]["gripper_feedback"][
            "spring_constant"
        ]

        self.gripper_spring_displacement_offset = self.config["controller"][
            "gripper_feedback"
        ]["spring_displacement_offset"]

        # exponential smoothing
        self.enable_exponential_smoothing = self.config["controller"][
            "exponential_smoothing"
        ]["enable"]
        self.smoothing_alpha = self.config["controller"]["exponential_smoothing"][
            "alpha"
        ]
        self.smoothed_torque_arm = np.zeros(self.num_arm_joints)
        self.smoothed_torque_gripper = 0.0

        # needs to be implemented to establish communication between the leader and the follower
        self.set_up_communication()

        # calibrate the leader arm joints before starting
        self._set_homing_offsets_in_docked_position()
        # ensure the leader and the follower arms have the same joint positions before starting
        self._match_start_pos()
        # start the control loop
        self.timer = self.create_timer(self.dt, self.control_loop_callback)

    def _set_homing_offsets_in_docked_position(self):
        """
        Set the homing offsets of each dynamixel such that the resultant position readings are:

        [0, -1.52, 0, -3.14, 0, 1.52, 1.52] (calibration_joint_pos in config.yaml)

        Should replace the _get_dynamixel_offsets() function by saving offset data to the EEPROMs directly.

        Before calling this function, manually place the leader arm in the calibration position.
        """
        ADDR_HOMING_OFFSET = 20

        # Ensure torque is disabled before writing to EEPROM
        torque_was_enabled = self.driver.torque_enabled
        if torque_was_enabled:
            self.driver.set_torque_mode(False)

        # Step 1: First, set all homing offsets to zero to get raw positions
        self.get_logger().info("Clearing existing homing offsets...")
        for i in range(self.num_motors):
            joint_id = i + 1
            dxl_comm_result, dxl_error = self.driver._packetHandler.write4ByteTxRx(
                self.driver._portHandler, joint_id, ADDR_HOMING_OFFSET, 0
            )

            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                raise RuntimeError(
                    f"Failed to clear homing offset for Dynamixel ID {joint_id}. "
                    f"Comm result: {dxl_comm_result}, Error: {dxl_error}"
                )

        # Warm up the driver after clearing offsets
        for _ in range(10):
            self.driver.get_positions_and_velocities()

        # Step 2: Get current positions (where Dynamixels think they are with zero offset)
        curr_joints, _ = self.driver.get_positions_and_velocities()

        # Step 3: Calculate error between current reading and actual calibration position
        # We need to find the offset that brings the current reading to the target,
        # accounting for multi-turn capability (positions can be > 2π)

        homing_offsets = []
        for i in range(self.num_motors):
            if i < self.num_arm_joints:
                # For arm joints
                current_reading = curr_joints[i] * self.joint_signs[i]
                target_reading = self.calibration_joint_pos[i]

                # Find the error, wrapping to the nearest equivalent angle
                # This handles the case where current_reading might be multiple revolutions away
                raw_error = target_reading - current_reading

                # Wrap error to [-π, π] by finding the equivalent angle closest to 0
                # This assumes the arm is within ±π of the target (within 180 degrees)
                wrapped_error = np.arctan2(np.sin(raw_error), np.cos(raw_error))

                # Sanity check: if the wrapped error is large, the arm might not be
                # in the calibration position
                if abs(wrapped_error) > np.pi / 2:  # More than 90 degrees off
                    self.get_logger().warn(
                        f"Joint {i+1} appears to be {np.degrees(abs(wrapped_error)):.1f} degrees "
                        f"away from calibration position. Please verify the arm is correctly positioned."
                    )

                # The homing offset should correct this error
                offset_rad = wrapped_error / self.joint_signs[i]
            else:
                # For gripper, set it to read 0 at current position
                offset_rad = -curr_joints[i]

            homing_offsets.append(offset_rad)

        # Step 4: Write homing offsets to EEPROM
        for i, offset_rad in enumerate(homing_offsets):
            # Convert offset from radians to Dynamixel units (1 unit = pi/2048 radians)
            offset_units = int(offset_rad * 2048.0 / np.pi)

            joint_id = i + 1
            dxl_comm_result, dxl_error = self.driver._packetHandler.write4ByteTxRx(
                self.driver._portHandler, joint_id, ADDR_HOMING_OFFSET, offset_units
            )

            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                raise RuntimeError(
                    f"Failed to set homing offset for Dynamixel ID {joint_id}. "
                    f"Comm result: {dxl_comm_result}, Error: {dxl_error}"
                )

            self.get_logger().info(
                f"Set homing offset for joint {joint_id}: {offset_rad:.4f} rad ({offset_units} units)"
            )

        # Step 5: Read positions again after writing homing offsets
        self.get_logger().info("Verifying homing offsets...")
        for _ in range(10):
            self.driver.get_positions_and_velocities()
        new_joints, _ = self.driver.get_positions_and_velocities()

        # Step 6: Verify that the error is now small
        max_error = 0.0
        for i in range(self.num_arm_joints):
            current_reading = new_joints[i] * self.joint_signs[i]
            # Wrap current reading to [-π, π]
            current_reading = np.arctan2(
                np.sin(current_reading), np.cos(current_reading)
            )

            target_reading = self.calibration_joint_pos[i]
            # Wrap target reading to [-π, π] as well for consistency
            target_reading = np.arctan2(np.sin(target_reading), np.cos(target_reading))

            # Calculate wrapped error
            error = target_reading - current_reading
            error_rad = abs(np.arctan2(np.sin(error), np.cos(error)))
            max_error = max(max_error, error_rad)

            assert error_rad < 0.1, (
                f"Joint {i+1} homing offset verification failed. "
                f"Error: {error_rad:.4f} rad ({np.degrees(error_rad):.2f} degrees). "
                f"Expected: {target_reading:.4f}, Got: {current_reading:.4f}"
            )

        # Re-enable torque if it was previously enabled
        if torque_was_enabled:
            self.driver.set_torque_mode(True)

        self.get_logger().info(
            f"Successfully set and verified homing offsets for all joints. "
            f"Max error: {max_error:.4f} rad ({np.degrees(max_error):.2f} degrees). "
            f"The offsets are now stored in EEPROM and will persist across power cycles."
        )

    def _prepare_dynamixel(self):
        """
        Instantiates driver for interfacing with Dynamixel servos.
        """
        self.servo_types = self.config["dynamixel"]["servo_types"]
        self.num_motors = len(self.servo_types)
        self.joint_signs = np.array(
            self.config["dynamixel"]["joint_signs"], dtype=float
        )
        assert self.num_motors == len(
            self.joint_signs
        ), "The number of motors and the number of joint signs must be the same"
        self.dynamixel_port = (
            "/dev/serial/by-id/" + self.config["dynamixel"]["dynamixel_port"]
        )

        # checks of the latency timer on ttyUSB of the corresponding port is 1
        # if it is not 1, the control loop cannot run at above 200 Hz, which will
        # cause extremely undesirable behaviour for the leader arm. If the latency
        # timer is not 1, one can set it to 1 as follows:
        # echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB{NUM}/latency_timer
        ttyUSBx = find_ttyusb(self.dynamixel_port)
        command = f"cat /sys/bus/usb-serial/devices/{ttyUSBx}/latency_timer"
        result = subprocess.run(
            command, shell=True, capture_output=True, text=True, check=True
        )
        ttyUSB_latency_timer = int(result.stdout)
        if ttyUSB_latency_timer != 1:
            raise Exception(
                f"Please ensure the latency timer of {ttyUSBx} is 1. Run: \n \
                echo 1 | sudo tee /sys/bus/usb-serial/devices/{ttyUSBx}/latency_timer"
            )

        joint_ids = np.arange(self.num_motors) + 1
        try:
            self.driver = DynamixelDriver(
                joint_ids, self.servo_types, self.dynamixel_port
            )
        except FileNotFoundError:
            self.get_logger().info(
                f"Port {self.dynamixel_port} not found. Please check the connection."
            )
            return
        self.driver.set_torque_mode(False)
        # set operating mode to current mode
        self.driver.set_operating_mode(0)
        # enable torque
        self.driver.set_torque_mode(True)

    def _prepare_inverse_dynamics(self):
        """
        Creates a model of the leader arm given the its URDF for kinematic and dynamic
        computations used in gravity compensation and null-space regulation calculations.
        """
        self.leader_urdf = os.path.join(
            "src/factr_teleop/factr_teleop/urdf/",
            self.config["arm_teleop"]["leader_urdf"],
        )

        # TODO WSH: DEJANK THESE HARDCODED PATHS
        self.pin_model, _, _ = pin.buildModelsFromUrdf(
            filename="/home/wheitman/wheitman_ws/src/factr_interface/build/factr_interface/factr_interface/urdf/factr_teleop_franka.urdf",
            package_dirs="/home/wheitman/wheitman_ws/src/factr_interface/build/factr_interface/factr_interface/urdf",
        )
        self.pin_data = self.pin_model.createData()

    def _match_start_pos(self):
        """
        Waits until the leader arm is manually moved to roughly the same configuration as the
        follower arm before the follower arm starts mirroring the leader arm.
        """
        # First, wait for the follower position to be available
        self.get_logger().info("Waiting for follower joint positions...")
        while self.latest_follower_positions is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not rclpy.ok():
            return  # Node is shutting down

        self.get_logger().info(
            "Follower positions received. Starting position matching..."
        )

        interpolation_step_size = (
            np.ones(7)
            * self.config["controller"]["joint_position_control"][
                "interpolation_step_size"
            ]
        )
        kp = self.config["controller"]["joint_position_control"]["kp"]
        kd = self.config["controller"]["joint_position_control"]["kd"]

        while rclpy.ok():
            curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel = (
                self.get_leader_joint_states()
            )
            next_joint_pos_target = np.where(
                np.abs(curr_pos - self.latest_follower_positions)
                > interpolation_step_size,
                curr_pos
                + interpolation_step_size
                * np.sign(self.latest_follower_positions - curr_pos),
                self.latest_follower_positions,
            )
            current_joint_error = np.linalg.norm(
                curr_pos - self.latest_follower_positions
            )
            np.set_printoptions(suppress=True, precision=4)
            self.get_logger().info(
                f"FACTR TELEOP {self.name}: Please match starting joint pos. Current joint pos: {curr_pos}"
            )

            self.get_logger().info(
                f"Errors {current_joint_error:.2f}: {curr_pos - self.latest_follower_positions}"
            )

            # Check if position is matched
            if np.all(np.abs(curr_pos - self.latest_follower_positions) < 0.2):
                break  # Each joint is close enough to expected start position!

            curr_pos, _, _, _ = self.get_leader_joint_states()

            goal_gripper_pos = 0.0  # TODO WSH: Read from follower

            torque = -kp * (curr_pos - next_joint_pos_target) - kd * (curr_vel)

            # gripper_torque = -kp * (curr_gripper_pos - goal_gripper_pos) - kd * (
            #     curr_gripper_vel
            # )

            gripper_torque = 0.0
            self.set_leader_joint_torque(torque, gripper_torque)
            time.sleep(0.1)

        self.get_logger().info(
            f"FACTR TELEOP {self.name}: Initial joint position matched."
        )

    def shut_down(self):
        """
        Disables all torque on the leader arm and gripper during node shutdown.
        """
        self.set_leader_joint_torque(np.zeros(self.num_arm_joints), 0.0)
        self.driver.set_torque_mode(False)

    def get_leader_joint_states(self):
        """
        Returns the current joint positions and velocities of the leader arm and gripper,
        aligned with the joint conventions (range and direction) of the follower arm.
        All positions are wrapped to [-π, π].
        """
        self.gripper_pos_prev = self.gripper_pos
        joint_pos, joint_vel = self.driver.get_positions_and_velocities()

        # Apply joint signs to get positions in the correct direction
        # Note: Homing offsets are now stored in EEPROM, so no software offset needed
        joint_pos_arm = (
            joint_pos[0 : self.num_arm_joints]
            * self.joint_signs[0 : self.num_arm_joints]
        )

        # Wrap all joint positions to [-π, π]
        for i in range(len(joint_pos_arm)):
            joint_pos_arm[i] = np.arctan2(
                np.sin(joint_pos_arm[i]), np.cos(joint_pos_arm[i])
            )

        self.gripper_pos = joint_pos[-1] * self.joint_signs[-1]

        joint_vel_arm = (
            joint_vel[0 : self.num_arm_joints]
            * self.joint_signs[0 : self.num_arm_joints]
        )

        gripper_vel = (self.gripper_pos - self.gripper_pos_prev) / self.dt
        return joint_pos_arm, joint_vel_arm, self.gripper_pos, gripper_vel

    def set_leader_joint_pos(self, goal_joint_pos, goal_gripper_pos):
        """
        Moves the leader arm and gripper to a specified joint configuration using a PD control loop.
        This method is useful for aligning the leader arm with a desired configuration, such as
        matching the follower arm's configuration. It interpolates the motion toward the target
        position and applies torque commands based on a PD controller.

        **Note:** This function is not used by default in the main teleoperation loop. To ensure
        controller stability, please ensure the latency of Dynamixel servos is minimized such
        that the control loop frequency is at least 200 Hz. Otherwise, the PD controller tuning
        is unstable for low control frequencies.
        """
        interpolation_step_size = (
            np.ones(7) * self.config["controller"]["interpolation_step_size"]
        )
        kp = self.config["controller"]["joint_position_control"]["kp"]
        kd = self.config["controller"]["joint_position_control"]["kd"]

        curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel = (
            self.get_leader_joint_states()
        )
        while np.linalg.norm(curr_pos - goal_joint_pos) > 0.1:
            next_joint_pos_target = np.where(
                np.abs(curr_pos - goal_joint_pos) > interpolation_step_size,
                curr_pos + interpolation_step_size * np.sign(goal_joint_pos - curr_pos),
                goal_joint_pos,
            )
            torque = -kp * (curr_pos - next_joint_pos_target) - kd * (curr_vel)
            gripper_torque = -kp * (curr_gripper_pos - goal_gripper_pos) - kd * (
                curr_gripper_vel
            )
            self.set_leader_joint_torque(torque, gripper_torque)
            curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel = (
                self.get_leader_joint_states()
            )

    def set_leader_joint_torque(self, arm_torque, gripper_torque):
        """
        Applies torque to the leader arm and gripper.
        """
        arm_gripper_torque = np.append(arm_torque, gripper_torque)
        self.driver.set_torque(arm_gripper_torque * self.joint_signs)

    def joint_limit_barrier(
        self, arm_joint_pos, arm_joint_vel, gripper_joint_pos, gripper_joint_vel
    ):
        """
        Computes joint limit repulsive torque to prevent the leader arm and gripper from
        exceeding the physical joint limits of the follower arm.

        This method implements a simplified control law compared to the one described in
        Section IX.B of the paper, while achieving the same protective effect. It applies
        repulsive torques proportional to the distance from the joint limits and the joint
        velocity when limits are approached or exceeded.
        """
        exceed_max_mask = arm_joint_pos > self.arm_joint_limits_max
        tau_l = (
            -self.joint_limit_kp * (arm_joint_pos - self.arm_joint_limits_max)
            - self.joint_limit_kd * arm_joint_vel
        ) * exceed_max_mask
        exceed_min_mask = arm_joint_pos < self.arm_joint_limits_min
        tau_l += (
            -self.joint_limit_kp * (arm_joint_pos - self.arm_joint_limits_min)
            - self.joint_limit_kd * arm_joint_vel
        ) * exceed_min_mask

        if gripper_joint_pos > self.gripper_limit_max:
            tau_l_gripper = (
                -self.joint_limit_kp * (gripper_joint_pos - self.gripper_limit_max)
                - self.joint_limit_kd * gripper_joint_vel
            )
        elif gripper_joint_pos < self.gripper_limit_min:
            tau_l_gripper = (
                -self.joint_limit_kp * (gripper_joint_pos - self.gripper_limit_min)
                - self.joint_limit_kd * gripper_joint_vel
            )
        else:
            tau_l_gripper = 0.0

        # NOTE: I removed joint limit feedback for the gripper here. WSH.
        tau_l_gripper = 0.0

        return tau_l, tau_l_gripper

    def gravity_compensation(self, arm_joint_pos, arm_joint_vel):
        """
        Computes joint torque for gravity compensation using inverse dynamics.
        This method uses the Recursive Newton-Euler Algorithm (RNEA), provided by the
        Pinocchio library, to calculate the torques required to counteract gravity
        at the current joint states. The result is scaled by a modifier to tune the
        compensation strength.

        This implementation corresponds to the gravity compensation strategy
        described in Section III.C of the paper.
        """
        self.tau_g = pin.rnea(
            self.pin_model,
            self.pin_data,
            arm_joint_pos,
            arm_joint_vel,
            np.zeros_like(arm_joint_vel),
        )
        self.tau_g *= self.gravity_comp_modifier
        return self.tau_g

    def friction_compensation(self, arm_joint_vel):
        """
        Compute joint torques to compensate for static friction during teleoperation.

        This method implements static friction compensation as described in Equation 7,
        Section IX.A of the paper. It omits kinetic friction compensation, which was
        necessary in earlier hardware versions to achieve smooth teleoperation, but has
        since become unnecessary due to hardware improvements, such as weight reduction.
        """
        tau_ss = np.zeros(self.num_arm_joints)
        for i in range(self.num_arm_joints):
            if abs(arm_joint_vel[i]) < self.stiction_comp_enable_speed:
                if self.stiction_dither_flag[i]:
                    tau_ss[i] += self.stiction_comp_gain * abs(self.tau_g[i])
                else:
                    tau_ss[i] -= self.stiction_comp_gain * abs(self.tau_g[i])
                self.stiction_dither_flag[i] = ~self.stiction_dither_flag[i]
        return tau_ss

    def null_space_regulation(self, arm_joint_pos, arm_joint_vel):
        """
        Computes joint torques to perform null-space regulation for redundancy resolution
        of the leader arm.

        This method enables the specification of a desired null-space joint configuration
        via `self.null_space_joint_target`. It implements the control strategy described
        in Equation 3 of Section III.B in the paper, projecting a PD control law into
        the null space of the task Jacobian to achieve secondary objectives without
        affecting the primary task.
        """
        J = pin.computeJointJacobian(
            self.pin_model, self.pin_data, arm_joint_pos, self.num_arm_joints
        )
        J_dagger = np.linalg.pinv(J)
        null_space_projector = np.eye(self.num_arm_joints) - J_dagger @ J
        q_error = arm_joint_pos - self.null_space_joint_target[0 : self.num_arm_joints]
        tau_n = null_space_projector @ (
            -self.null_space_kp * q_error - self.null_space_kd * arm_joint_vel
        )
        return tau_n

    def torque_feedback(self, external_torque, arm_joint_vel):
        """
        Computes joint torque for the leader arm to achieve force-feedback based on
        the external joint torque from the follower arm.

        This method implements Equation 1 in Section III.A of the paper.
        """
        tau_ff = (
            -1.0
            * self.torque_feedback_gain
            / self.torque_feedback_motor_scalar
            * external_torque
        )
        tau_ff -= self.torque_feedback_damping * arm_joint_vel
        return tau_ff

    def control_loop_callback(self):
        """
        Runs the main control loop of the leader arm.

        Note that while the control loop can run at up to 500 Hz, lower frequencies
        such as 200 Hz can still yield comparable performance, although they may
        require additional tuning of control parameters. For Dynamixel servos to
        support a 500 Hz control frequency, ensure that the Baud Rate is set to 4 Mbps
        and the Return Delay Time is set to 0 using the Dynamixel Wizard software.
        """

        leader_arm_pos, leader_arm_vel, leader_gripper_pos, leader_gripper_vel = (
            self.get_leader_joint_states()
        )

        torque_arm = np.zeros(self.num_arm_joints)
        torque_l, torque_gripper = self.joint_limit_barrier(
            leader_arm_pos, leader_arm_vel, leader_gripper_pos, leader_gripper_vel
        )
        # print(f"JL Barrier {torque_gripper}")
        torque_arm += torque_l

        torque_null_space = self.null_space_regulation(leader_arm_pos, leader_arm_vel)
        # print(f"NS Reg {torque_null_space}")
        torque_arm += torque_null_space

        if self.enable_gravity_comp:
            torque_gravity_comp = self.gravity_compensation(
                leader_arm_pos, leader_arm_vel
            )
            torque_friction_comp = self.friction_compensation(leader_arm_vel)

            # print(f"Grav Comp {torque_gravity_comp}")
            # print(f"Friction Comp {torque_friction_comp}")

            torque_arm += torque_gravity_comp
            torque_arm += torque_friction_comp

        if self.enable_torque_feedback:
            external_joint_torque = self.get_leader_arm_external_joint_torque()
            torque_feedback = self.torque_feedback(
                external_joint_torque, leader_arm_vel
            )

            # Only consider the second joint
            torque_feedback[4:] = 0.0

            print(f"Torque FB {torque_feedback}")

            self.latest_leader_torques = torque_feedback
            torque_arm += torque_feedback

        if self.enable_gripper_feedback:
            gripper_feedback = self.get_leader_gripper_feedback()
            torque_gripper += self.gripper_feedback(
                leader_gripper_pos, leader_gripper_vel, gripper_feedback
            )

        if np.any(np.abs(torque_arm) > self.global_torque_limit):
            self.get_logger().warning("Global torque limit exceeded!")

        torque_arm = np.clip(
            torque_arm, -self.global_torque_limit, self.global_torque_limit
        )

        # Apply exponential smoothing if enabled
        if self.enable_exponential_smoothing:
            self.smoothed_torque_arm = (
                self.smoothing_alpha * torque_arm
                + (1 - self.smoothing_alpha) * self.smoothed_torque_arm
            )
            self.smoothed_torque_gripper = (
                self.smoothing_alpha * torque_gripper
                + (1 - self.smoothing_alpha) * self.smoothed_torque_gripper
            )

            # self.latest_leader_torques = np.concatenate(
            #     [self.smoothed_torque_arm.copy(), [self.smoothed_torque_gripper]]
            # )

            self.set_leader_joint_torque(
                self.smoothed_torque_arm, self.smoothed_torque_gripper
            )
        else:
            self.latest_leader_torques = np.concatenate(
                [torque_arm.copy(), [torque_gripper]]
            )
            self.set_leader_joint_torque(torque_arm, torque_gripper)

        self.update_communication(leader_arm_pos, leader_gripper_pos)

    @abstractmethod
    def set_up_communication(self):
        """
        This method should be implemented to set up communication between the leader arm
        and the follower arm for bilateral teleoperation. This method is called once
        in the __init__ method.

        For example, a subscriber can  be set up to receive external joint torque from
        the leader arm and a publisher can be set up to send joint position target commands
        to the follower arm. Publishers and subscribers can also be set up to record
        the follower arm's joint states

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass

    @abstractmethod
    def get_leader_arm_external_joint_torque(self) -> np.ndarray:
        """
        This method should retrieve the current external joint torque from the follower arm.
        This is used to compute force-feedback in the leader arm. This method is called at
        every iteration of the control loop if self.enable_torque_feedback is set to True.

        Returns:
            np.ndarray: A NumPy array of shape (num_arm_joints,) containing the external
            joint torques.

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass

    @abstractmethod
    def get_leader_gripper_feedback(self):
        """
        This method should retrieve any data from the follower gripper that might be required
        to achieve force-feedback in the leader gripper. For example, this method can be used
        to get the current position of the follower gripper for position-position force-feedback
        or the current force of the follower gripper for position-force force-feedback in the
        leader gripper. This method is called at every iteration of the control loop if
        self.enable_gripper_feedback is set to True.

        Returns:
            Any: Feedback data required by the leader gripper. This can be a NumPy array, a
            scalar, or any other data type depending on the implementation.

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass

    @abstractmethod
    def gripper_feedback(
        self, leader_gripper_pos, leader_gripper_vel, gripper_feedback
    ):
        """
        Processes feedback data from the follower gripper. This method is intended to compute
        force-feedback for the leader gripper. This method is called at every iteration of the
        control loop if self.enable_gripper_feedback is set to True.

        Args:
            leader_gripper_pos (float): Leader gripper position. Can be used to provide force-
            feedback for the gripper.
            leader_gripper_vel (float): Leader gripper velocity. Can be used to provide force-
            feedback for the gripper.
            gripper_feedback (Any): Feedback data from the gripper. The format can vary depending
            on the implementation, such as a NumPy array, scalar, or custom object.

        Returns:
            float: The computed joint torque value to apply force-feedback to the leader gripper.

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass

    @abstractmethod
    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        """
        This method is intended to be called at every iteration of the control loop to transmit
        relevant data, such as joint position targets, from the leader to the follower arm.

        Args:
            leader_arm_pos (np.ndarray): A NumPy array containing the joint positions of the leader arm.
            leader_gripper_pos (np.ndarray): A NumPy array containing the position of the leader gripper.

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """

        pass
