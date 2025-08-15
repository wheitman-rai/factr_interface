# -----------------------------------------------------------------------------
# Description: Bridges data from Dynamixel motors and robot arms (Franka, Kinova)
# Author: Will Heitman
# (c) 2025 RAI Institute
# -----------------------------------------------------------------------------

from io import TextIOWrapper
import os
from typing import List
import yaml

from ament_index_python.packages import get_package_share_directory
from dynamixel_sdk import (
    PacketHandler,
    Protocol2PacketHandler,
    PortHandler,
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
)
import numpy as np
import math
import rclpy
from rclpy.impl import rcutils_logger
from rclpy.action import ActionClient
from rclpy.node import Node, ParameterDescriptor, ParameterType
from time import time


# Messages
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

# See https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#control-table-of-ram-area
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_POSITION = 116
ADDR_GOAL_CURRENT = 102  # This is the address for Goal Current
ADDR_OPERATING_MODE = 11
ADDR_CURRENT_LIMIT = 38  # This address may vary by model.
LEN_PRESENT_POSITION = 4  # 4 bytes for position data
LEN_GOAL_POSITION = 4  # 4 bytes for position data
ADDR_TORQUE_ENABLE = 64


class Dynamixel:

    def __init__(
        self,
        id: int,
        packet_handler: Protocol2PacketHandler,
        port_handler: PortHandler,
    ):

        self.packet_handler = packet_handler
        self.port_handler = port_handler
        self.id = id
        self.offset_radians: float = 0.0  # "Zero" position (homing pos) of the motor
        self.logger = rcutils_logger.RcutilsLogger(name="factr")

        # Ping the motor
        dxl_model_number, dxl_comm_result, dxl_error = packet_handler.ping(
            port_handler, id
        )
        if dxl_comm_result != COMM_SUCCESS:
            if "no status packet" in str(packet_handler.getTxRxResult(dxl_comm_result)):
                self.logger.error(
                    f"Could not communicate with Dynamixel {self.id}. Is the motor plugged in and powered?"
                )
            else:
                self.logger.error(f"{packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print("%s" % packet_handler.getRxPacketError(dxl_error))

        # else: success!

    @property
    def pos(self):
        dxl_present_position, dxl_comm_result, dxl_error = (
            self.packet_handler.read4ByteTxRx(
                self.port_handler, self.id, ADDR_PRESENT_POSITION
            )
        )
        if dxl_comm_result != COMM_SUCCESS:
            if "no status packet" in str(
                self.packet_handler.getTxRxResult(dxl_comm_result)
            ):
                print(f"Lost communication with motor {self.id}. Check wiring.")
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))

        pos_raw = dxl_present_position % 4096  # Limit to one rotation
        pos_degrees = pos_raw * 0.087891
        pos_radians = pos_degrees * np.pi / 180.0
        pos_radians_with_offset = pos_radians - self.offset_radians
        return pos_radians_with_offset

    def cal(self):
        self.offset_radians = self.pos


class Factr:

    def __init__(
        self,
        ids: List[int] = list(range(1, 9)),
        port: str = "/dev/ttyUSB0",
        baud: int = 57600,
    ):

        self.logger = rcutils_logger.RcutilsLogger(name="factr")

        # Establish a connection
        self.port_handler = PortHandler(port)
        self.packet_handler = Protocol2PacketHandler()

        # Open port
        if self.port_handler.openPort():
            self.logger.info(f"Connected to FACTR on port {port}")
        else:
            self.logger.error(f"Failed to connect to FACTR on port {port}")

        # Set port baudrate
        if self.port_handler.setBaudRate(baud):
            self.logger.info(f"FACTR's baud rate is now {baud}")
        else:
            self.logger.error(f"Failed to change FACTR's baud rate")

        # Create groups to efficienty read and write from multiple motors at once
        self.sync_read_group = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )

        # Instantiate 8 Dynamixels
        self.dynamixels: List[Dynamixel] = []
        for id in ids:
            self.dynamixels.append(
                Dynamixel(id, self.packet_handler, self.port_handler)
            )

            # Add the motor to the Sync Read Group
            if not self.sync_read_group.addParam(id):
                self.logger.error(f"Failed to add motor {id} to the sync read group")

    @property
    def pos(self):
        """Returns the position of each motor in radians, accounting for offsets from calibration

        Returns:
            List[float]: positions of each motor in radians
        """
        positions: List[float] = []

        # Read the position of each motor in the group
        dxl_comm_result = self.sync_read_group.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}")

        for dynamixel in self.dynamixels:

            raw_position = self.sync_read_group.getData(
                dynamixel.id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )

            # Limit to 4096 (one full rotation)
            raw_position = raw_position % 4096
            position_degrees = raw_position * 0.087891

            position_radians = position_degrees * np.pi / 180.0
            position_radians_with_offset = position_radians - dynamixel.offset_radians

            # Limit to [-180, 180] degrees
            if position_radians_with_offset < -np.pi:
                position_radians_with_offset += 2 * np.pi
            elif position_radians_with_offset > np.pi:
                position_radians_with_offset -= 2 * np.pi

            positions.append(position_radians_with_offset)

        return positions

    def calibrate_from_file(self):
        try:
            # Get the path to the package's share directory
            package_share_directory = get_package_share_directory("factr_interface")

            # Construct the full path to the config file
            config_file_path = os.path.join(
                package_share_directory, "config", "calibration.yaml"
            )

            # Open and read the YAML file
            with open(self.calibration_file_path, "r") as file:

                config_data: dict = yaml.safe_load(file)

                offsets_radians = config_data.get("offsets_radians")

                if offsets_radians is None:
                    self.logger.error(
                        "Could not retreive offsets_radians from calibration file. FACTR needs to be calibrated."
                    )
                elif len(offsets_radians) != 8:
                    self.logger.error(
                        "Expected offsets_radians to be an array of 8 floats"
                    )
                else:
                    print(offsets_radians)

                    for dynamixel, position in zip(self.dynamixels, offsets_radians):
                        dynamixel.offset_radians = position

                    self.logger.info(
                        "Successfully loaded FACTR's position offsets from the calibration file"
                    )
        except FileNotFoundError:
            print(f"Error: Config file not found at {config_file_path}")  # type: ignore
            return None

    @property
    def calibration_file_path(self) -> str:
        # Get the path to the package's share directory
        package_share_directory = get_package_share_directory("factr_interface")

        # Construct the full path to the config file
        return os.path.join(package_share_directory, "config", "calibration.yaml")

    def cal(self, save_to_file=True):
        # Move FACTR straight up, then set all position offsets to zero
        for dynamixel in self.dynamixels:
            dynamixel.cal()

        if save_to_file:
            offsets_radians = []

            for dynamixel in self.dynamixels:
                offsets_radians.append(dynamixel.offset_radians)

            with open(self.calibration_file_path, "w") as file:
                yaml.dump({"offsets_radians": offsets_radians}, file)
                self.logger.info(
                    "Successfully saved FACTR's position offsets to the calibration file"
                )


class FactrInterfaceNode(Node):
    """
    Bridges data from Dynamixel motors and robot arms (Franka, Kinova)
    """

    def __init__(self):
        super().__init__("factr_interface")

        self.follower_joint_state = None
        self.follower_joint_angles = None
        self.follower_at_zero = False
        self.factr_and_follower_initially_synced = False
        self.zero_angles = np.zeros(8)

        self.declare_parameters(
            "",
            [
                ("serial_port", "/dev/ttyUSB0", ParameterDescriptor()),
                ("dynamixel_ids", list(range(1, 9)), ParameterDescriptor()),
            ],
        )

        self.joint_state_pub = self.create_publisher(
            JointState, "/kinova/joint_commands", 10
        )

        self.joint_command_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 1
        )

        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 1)
        self.gripper_client = ActionClient(
            self, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )

        # self.set_up_motors()

        self.factr = Factr()

        # input("Move FACTR to zero position, then press Enter...")
        # self.factr.cal()

        self.factr.calibrate_from_file()

        while True:
            for pos in self.factr.pos:
                print(f"{pos:+03.2f}")

        self.create_timer(0.04, self.spin_interface)

        self.time_last_state_received = 0.0

    def joint_state_cb(self, msg: JointState) -> None:
        self.time_last_state_received = time()

        # Form a dictionary for joints
        self.follower_joint_state = {}

        self.follower_joint_angles = np.zeros(8)

        for idx, name in enumerate(msg.name):
            self.follower_joint_state[name] = {
                "position": msg.position[idx],
                "velocity": msg.velocity[idx],
                "effort": msg.effort[idx],
            }

            if "robotiq" in name:
                self.follower_joint_angles[7] = msg.position[idx]

            elif "joint_" in name:
                joint_index = int(name[-1]) - 1
                self.follower_joint_angles[joint_index] = msg.position[idx]

        self.follower_at_zero = np.linalg.norm(self.follower_joint_angles) < 1.0

        # print(self.follower_joint_state)

    def set_up_motors(self) -> None:
        """
        Initializes and configures the Dynamixel motors for communication.
        Opens the serial port, sets the baud rate, and adds each motor ID to the sync read group.
        Logs errors if connection or configuration fails.
        """

        self.port = self.get_parameter("serial_port").value
        self.dynamixel_ids: List[int] = self.get_parameter("dynamixel_ids").value

        self.port_handler = PortHandler(self.port)  # TODO WSH: Parameterize
        self.packet_handler: Protocol2PacketHandler = PacketHandler(protocol_version=2.0)  # type: ignore
        self.sync_read_group = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )

        # self.sync_write_group = GroupSyncWrite(
        #     self.port_handler,
        #     self.packet_handler,
        #     ADDR_GOAL_POSITION,
        #     LEN_GOAL_POSITION,
        # )

        if self.port_handler.openPort():
            print(f"Successfully connected to FACTR on port {self.port}")
        else:
            self.get_logger().error(f"Could not connect to FACTR on port {self.port}")

        if not self.port_handler.setBaudRate(57600):
            self.get_logger().error(f"Could not set baud rate on port {self.port}")

        self.motors = [
            Dynamixel(id, self.packet_handler, self.port_handler) for id in range(1, 9)
        ]

        for motor in self.motors:
            print(motor.pos)

        input("Press enter when ready to calibrate...")

        for motor in self.motors:
            motor.cal()

        while True:
            for motor in self.motors:
                print(motor.pos)

        quit()

        # Add all motors to the Sync Read Group
        # for dynamixel_id in self.dynamixel_ids:
        #     successful = self.sync_read_group.addParam(dynamixel_id)

        #     if not successful:
        #         self.get_logger().error(
        #             f"Failed to add motor {dynamixel_id} to the sync read group"
        #         )

        # Set zero position
        # self.get_logger().info("Hold FACTR in the zero position and press enter")
        # input("Press enter when ready...")
        # self.zero_angles = self.fetch_factr_angles()

        # Enable torques
        # print(f"Holding at {self.zero_angles}")
        # self.set_factr_goals(self.zero_angles, np.ones(8) * 45)

    def set_factr_goals(self, angles: np.ndarray, currents: np.ndarray):

        for id in range(1, 9):  # Dynamixel IDs

            # Disable torque if enabled
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, id, ADDR_TORQUE_ENABLE, 0
            )

            # Set operating mode to current-based position control
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, id, ADDR_OPERATING_MODE, 5
            )
            if dxl_comm_result != COMM_SUCCESS:
                print(self.packet_handler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print(self.packet_handler.getRxPacketError(dxl_error))
            else:
                print("Operating mode set to Current-based Position Control.")

            # Enable torque
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, id, ADDR_TORQUE_ENABLE, 1
            )

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packet_handler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" % id)

            # Write the Goal Position
            pos = int(
                angles[id - 1] / 0.087891
            )  # Dynamixel motor's typical resolution, in deg/tick
            self.packet_handler.write4ByteTxRx(
                self.port_handler, id, ADDR_GOAL_POSITION, pos
            )

            # Write the Goal Current
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler, id, ADDR_GOAL_CURRENT, int(currents[id - 1])
            )

            if dxl_comm_result != COMM_SUCCESS:
                print(self.packet_handler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print(self.packet_handler.getRxPacketError(dxl_error))
            else:
                print("Goal position sent successfully.")

    def enable_motor_torque(self, id: int, limit: int = 50):
        assert id >= 1 and id <= 8, "ID must be between 1 and 8"

        # Disable torque if enabled
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, id, ADDR_TORQUE_ENABLE, 0
        )

        # Set operating mode to current-based position control
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, id, ADDR_OPERATING_MODE, 5
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print(self.packet_handler.getRxPacketError(dxl_error))
        else:
            print("Operating mode set to Current-based Position Control.")

        # Enable torque
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, id, ADDR_TORQUE_ENABLE, 1
        )
        # dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
        #     self.port_handler, id, ADDR_CURRENT_LIMIT, limit
        # )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % id)

        GOAL_POSITION = 2048  # Midpoint
        GOAL_CURRENT = 80  # Low current value for compliant movement

        # Write the Goal Position
        self.packet_handler.write4ByteTxRx(
            self.port_handler, id, ADDR_GOAL_POSITION, GOAL_POSITION
        )
        # Write the Goal Current
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, id, ADDR_GOAL_CURRENT, GOAL_CURRENT
        )

        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print(self.packet_handler.getRxPacketError(dxl_error))
        else:
            print("Goal position sent successfully.")

    def map(self, factr_min, factr_max, follower_min, follower_max, value) -> float:
        factr_range = factr_max - factr_min

        follower_range = follower_max - follower_min

        follower_value = (value - factr_min) / factr_range * follower_range

        # Clip to follower limits
        follower_value = max(follower_value, follower_min)
        follower_value = min(follower_value, follower_max)

        return follower_value

    def spin_interface(self) -> None:

        # if not self.factr_and_follower_initially_synced:

        #     if not self.follower_at_zero:
        #         self.get_logger().warning(
        #             f"Follower must be at zero position to start."
        #         )
        #         return

        if self.follower_joint_angles is None:
            self.get_logger().warning(
                f"No joint state received from follower. Skipping controller update."
            )
            return
        elif len(self.follower_joint_angles) != 8:
            self.get_logger().error(
                f"Expected 8 follower angles, but got {len(self.follower_joint_angles)}"
            )
            return

        self.fetch_factr_angles(radians=True, offset=True)

        self.publish_joint_state()

        gripper_position = self.map(-1, 0.0, 0.0, 1.0, self.factr_angles[7])
        self.factr_angles[7] = gripper_position

        command_msg = JointTrajectory()
        command_msg.joint_names = [f"joint_{i}" for i in range(1, 8)]

        position_error = self.factr_angles - self.follower_joint_angles

        if (
            not self.factr_and_follower_initially_synced
            and np.linalg.norm(position_error) > 0.5
        ):
            self.get_logger().warning(f"Align FACTR with follower to start.")
            self.get_logger().warning(f"{position_error}")
            return

        Kp = np.zeros_like(position_error) * 10.0

        print(position_error)

        # Control the gripper
        gripper_command = GripperCommand.Goal()
        gripper_command.command.position = gripper_position
        gripper_command.command.max_effort = 100.0
        gripper_goal_future_ = self.gripper_client.send_goal_async(gripper_command)

        # And the other joints...

        norm_joint_position_error = np.linalg.norm(position_error)
        print(norm_joint_position_error)
        time_from_start = norm_joint_position_error

        trajectory_point = JointTrajectoryPoint()
        follower_positions = np.zeros(7)
        follower_positions[3:] = self.factr_angles[3:7]
        trajectory_point.positions = list(follower_positions)
        trajectory_point.time_from_start = Duration(
            sec=int(np.floor(time_from_start)),
            nanosec=int((time_from_start - np.floor(time_from_start)) * 1e9),
        )
        command_msg.points = [trajectory_point]

        self.joint_command_pub.publish(command_msg)

    # def nudge_toward_config(angles: np.ndarray, max_torques = np.ndarray)

    def fetch_factr_angles(self, radians=False, offset=False) -> np.ndarray:

        start = time()

        self.factr_angles = []

        # Read the position of each motor in the group
        dxl_comm_result = self.sync_read_group.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}")
            exit()

        for idx, dynamixel_id in enumerate(self.dynamixel_ids):

            pos = self.sync_read_group.getData(
                dynamixel_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )

            print(f"{idx} : {pos}")

            # Handle signed integers from Dynamixel
            # if pos > 2**31:
            #     pos -= 2**32

            # Get position in DEGREES
            pos *= 0.087891  # Dynamixel motor's typical resolution, in deg/tick

            if offset:
                # Offset the position relative to zero position
                # (FACTR pointing straight up)
                pos -= self.zero_angles[idx]

            if radians:
                pos *= np.pi / 180.0

            self.factr_angles.append(pos)

        print(f"Took {time() - start} secs")

        return np.asarray(self.factr_angles, dtype=np.float32)

    def publish_joint_state(self) -> None:
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.position = [
            float(pos) for pos in self.factr_angles
        ]  # TODO: Perform proper conversion!

        self.joint_state_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = FactrInterfaceNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
