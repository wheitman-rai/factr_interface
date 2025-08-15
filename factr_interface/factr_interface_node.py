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
from time import time, sleep


# Messages
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

# See https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#control-table-of-ram-area
ADDR_GOAL_POSITION = 116
ADDR_GOAL_CURRENT = 102  # This is the address for Goal Current
ADDR_HOMING_OFFSET = 20
ADDR_PRESENT_POSITION = 132
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
        pos_radians_with_offset = pos_radians
        return pos_radians_with_offset

    def cal(self):
        """Calibrate the Dynamixel by setting its homing offset"""

        # 1. Disable torque before changing the operating mode
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.id, ADDR_TORQUE_ENABLE, 0
        )

        # 2. Set the Operating Mode to Current-based Position Control (Value 5)
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.id, ADDR_OPERATING_MODE, 5
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))
            return
        elif dxl_error != 0:
            print(self.packet_handler.getRxPacketError(dxl_error))
            return

        # 3. Get the current position
        dxl_present_position, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.id, ADDR_PRESENT_POSITION
        )

        print(f"Pos was: {dxl_present_position}")

        if result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(result))
            return
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return

        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.id, ADDR_HOMING_OFFSET, 0
        )

        dxl_present_position, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.id, ADDR_PRESENT_POSITION
        )

        # 4. Write dxl_present_position to the negative of its homing offset
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.id, ADDR_HOMING_OFFSET, -dxl_present_position
        )

        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))
            return
        elif dxl_error != 0:
            print(self.packet_handler.getRxPacketError(dxl_error))
            return

        # 7. Read the new position after the reboot
        dxl_present_position, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.id, ADDR_PRESENT_POSITION
        )

        if result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(result))
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
        else:
            print(
                f"Pos is now: {dxl_present_position}"
            )  # Should be 0 or very close to it

    def spring_to(self, goal_pos_radians: float, goal_current_ma: int = 100):
        """
        Sets the goal current and goal pose of a Dynamixel, allowing virtual springs



        """

        # 1. Disable torque before changing the operating mode
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.id, ADDR_TORQUE_ENABLE, 0
        )
        print("Torque disabled.")

        # 2. Set the Operating Mode to Current-based Position Control (Value 5)
        # This value may need to be stored in the EEPROM and may require a reboot
        # to take effect permanently, but can be set in RAM for a single session.
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.id, ADDR_OPERATING_MODE, 5
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print(self.packet_handler.getRxPacketError(dxl_error))
        else:
            print("Operating mode set to Current-based Position Control.")

        # 3. Enable motor torque
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.id, ADDR_TORQUE_ENABLE, 1
        )
        print("Torque enabled.")

        # 4. Set a goal position and goal current
        # Range for goal current is typically 0 to 2047 (relative to maximum current)

        # Write the Goal Position
        raw_goal_pos = int(goal_pos_radians * 360 / np.pi / 0.087891)
        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.id, ADDR_GOAL_POSITION, raw_goal_pos
        )
        # Write the Goal Current
        self.packet_handler.write2ByteTxRx(
            self.port_handler, self.id, ADDR_GOAL_CURRENT, goal_current_ma
        )


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

    def spring_to_home(self) -> None:
        """Torque each motor to bias it toward the home position"""

        for dynamixel in self.dynamixels:
            if dynamixel.id in [1, 4]:
                dynamixel.spring_to(0.0, 100)
            elif dynamixel.id in [2]:
                dynamixel.spring_to(0.0, 300)
            else:
                dynamixel.spring_to(0.0, 50)

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

    def calibrate_from_file(self) -> None:
        """Load position offsets from calibration.yaml"""

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
        """Overwrite the position offsets, and optionally save them to calibration.yaml

        Args:
            save_to_file (bool, optional): Whether to save to calibration.yaml. Defaults to True.
        """

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

        self.factr = Factr()

        input("Move FACTR to zero position, then press Enter...")
        self.factr.cal()

        # Load position offsets from calibration.yaml
        self.factr.calibrate_from_file()

        self.factr.spring_to_home()


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

    def set_up_motors(self) -> None:
        """
        Initializes and configures the Dynamixel motors for communication.
        Opens the serial port, sets the baud rate, and adds each motor ID to the sync read group.
        Logs errors if connection or configuration fails.
        """

        self.port = self.get_parameter("serial_port").value
        self.dynamixel_ids: List[int] = self.get_parameter("dynamixel_ids").value  # type: ignore

        self.port_handler = PortHandler(self.port)  # TODO WSH: Parameterize
        self.packet_handler: Protocol2PacketHandler = PacketHandler(protocol_version=2.0)  # type: ignore
        self.sync_read_group = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )

        self.sync_write_group = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )

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

    def map(self, factr_min, factr_max, follower_min, follower_max, value) -> float:
        factr_range = factr_max - factr_min

        follower_range = follower_max - follower_min

        follower_value = (value - factr_min) / factr_range * follower_range

        # Clip to follower limits
        follower_value = max(follower_value, follower_min)
        follower_value = min(follower_value, follower_max)

        return follower_value

    def spin_interface(self) -> None: ...


def main(args=None):

    rclpy.init(args=args)

    node = FactrInterfaceNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
