# -----------------------------------------------------------------------------
# Description: Bridges data from Dynamixel motors and robot arms (Franka, Kinova)
# Author: Will Heitman
# (c) 2025 RAI Institute
# -----------------------------------------------------------------------------

from functools import partial
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
from robot_descriptions.loaders.yourdfpy import load_robot_description
from time import time, sleep
import viser
from viser.extras import ViserUrdf
import yourdfpy


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
        inverted=False,
    ):

        self.packet_handler = packet_handler
        self.port_handler = port_handler
        self.id = id
        self.offset_radians: float = 0.0  # "Zero" position (homing pos) of the motor
        self.logger = rcutils_logger.RcutilsLogger(name="factr")
        self.inverted = inverted

        # Ping the motor
        dxl_model_number, dxl_comm_result, dxl_error = packet_handler.ping(
            port_handler, id
        )
        if dxl_comm_result != COMM_SUCCESS:
            if "no status packet" in str(packet_handler.getTxRxResult(dxl_comm_result)):
                self.logger.error(
                    f"Could not communicate with Dynamixel {self.id}. Is the motor plugged in and powered? Is the baud rate correct?"
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
        return (
            pos_radians_with_offset * -1 if self.inverted else pos_radians_with_offset
        )

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

            if abs(dxl_present_position) > 10:
                self.logger.warning(
                    f"Motor {self.id} has position {dxl_present_position} after calibration!"
                )

    def spring_to(
        self, goal_pos_radians: float, goal_current_ma: int = 100, initialize=False
    ):
        """
        Sets the goal current and goal pose of a Dynamixel, allowing virtual springs



        """

        if initialize:
            # 1. Disable torque before changing the operating mode
            self.packet_handler.write1ByteTxRx(
                self.port_handler, self.id, ADDR_TORQUE_ENABLE, 0
            )

            # 2. Set the Operating Mode to Current-based Position Control (Value 5)
            # This value may need to be stored in the EEPROM and may require a reboot
            # to take effect permanently, but can be set in RAM for a single session.
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self.id, ADDR_OPERATING_MODE, 5
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.logger.error(self.packet_handler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self.logger.error(self.packet_handler.getRxPacketError(dxl_error))

            # 3. Enable motor torque
            self.packet_handler.write1ByteTxRx(
                self.port_handler, self.id, ADDR_TORQUE_ENABLE, 1
            )

        # 4. Set a goal position and goal current
        # Range for goal current is typically 0 to 2047 (relative to maximum current)

        # Write the Goal Position
        raw_goal_pos = int(goal_pos_radians * 180 / np.pi / 0.087891)
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
        baud: int = 4000000,
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

        self.sync_write_pos = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )

        self.sync_write_current = GroupSyncWrite(
            self.port_handler, self.packet_handler, ADDR_GOAL_CURRENT, 2
        )

        self.inverted = [False, False, False, True, False, True, False, False]

        # Instantiate 8 Dynamixels
        self.dynamixels: List[Dynamixel] = []
        for id, inverted in zip(ids, self.inverted):
            self.dynamixels.append(
                Dynamixel(id, self.packet_handler, self.port_handler, inverted)
            )

            # Add the motor to the Sync Read Group
            if not self.sync_read_group.addParam(id):
                self.logger.error(f"Failed to add motor {id} to the sync read group")

    def spring_to(self, positions, currents, initialize=False):
        """
        Sets the goal position and goal current of all Dynamixels at once
        using GroupSyncWrite.
        """
        start = time()

        if initialize:
            # 1. Disable torque before changing the operating mode for all motors
            self.cut_torque()

            # 2. Set the Operating Mode to Current-based Position Control (Value 5)
            for dynamixel in self.dynamixels:
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, dynamixel.id, ADDR_OPERATING_MODE, 5
                )

            # 3. Enable motor torque for all motors
            for dynamixel in self.dynamixels:
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, dynamixel.id, ADDR_TORQUE_ENABLE, 1
                )

        # Clear existing parameters
        self.sync_write_pos.clearParam()
        self.sync_write_current.clearParam()

        # Add goal positions and currents for all motors
        for dynamixel, position, current in zip(self.dynamixels, positions, currents):
            # Convert radians to raw position data (4 bytes)
            raw_goal_pos = int(position * 180 / np.pi / 0.087891)
            param_goal_position = [
                (raw_goal_pos >> 0) & 0xFF,
                (raw_goal_pos >> 8) & 0xFF,
                (raw_goal_pos >> 16) & 0xFF,
                (raw_goal_pos >> 24) & 0xFF,
            ]

            # Convert current to raw data (2 bytes)
            param_goal_current = [
                (current >> 0) & 0xFF,
                (current >> 8) & 0xFF,
            ]

            # Add parameters to the group sync write objects
            self.sync_write_pos.addParam(dynamixel.id, param_goal_position)
            self.sync_write_current.addParam(dynamixel.id, param_goal_current)

        # Transmit the data to all motors at once
        dxl_comm_result_pos = self.sync_write_pos.txPacket()
        dxl_comm_result_current = self.sync_write_current.txPacket()

        if dxl_comm_result_pos != COMM_SUCCESS:
            print(
                f"Goal position sync write failed: {self.packet_handler.getTxRxResult(dxl_comm_result_pos)}"
            )
        if dxl_comm_result_current != COMM_SUCCESS:
            print(
                f"Goal current sync write failed: {self.packet_handler.getTxRxResult(dxl_comm_result_current)}"
            )

        # print(f"Factr.spring_to: {time() - start}")

    def spring_to_home(self, initialize=False) -> None:
        """Torque each motor to bias it toward the home position"""

        for dynamixel in self.dynamixels:
            if dynamixel.id in [1, 4]:
                dynamixel.spring_to(0.0, 60, initialize=initialize)
            elif dynamixel.id in [2]:
                dynamixel.spring_to(0.0, 100, initialize=initialize)
            else:
                dynamixel.spring_to(0.0, 30, initialize=initialize)

    def cut_torque(self):
        for dynamixel in self.dynamixels:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, dynamixel.id, ADDR_TORQUE_ENABLE, 0
            )

    @property
    def pos(self) -> np.ndarray:
        """Returns the position of each motor in radians, accounting for offsets from calibration

        Returns:
            np.ndarray: positions of each motor in radians
        """
        start = time()

        # Simple caching mechanism to use the last result if newer than x seconds
        if (
            hasattr(self, "time_since_last_pos_reading_")
            and time() - self.time_since_last_pos_reading_ < 0.025
        ):
            return self.last_pos_reading_

        else:
            self.time_since_last_pos_reading_ = time()

        positions: List[float] = []

        # Read the position of each motor in the group
        dxl_comm_result = self.sync_read_group.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}")

        for dynamixel, inverted in zip(self.dynamixels, self.inverted):

            raw_position: int = self.sync_read_group.getData(
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

            if inverted:
                position_radians_with_offset *= -1

            positions.append(position_radians_with_offset)

        # print(f"Factr.pos: {time() - start}")

        positions_np = np.asarray(positions)

        # Remap the gripper pos
        positions_np[-1] += 0.8

        self.last_pos_reading_ = positions_np

        return positions_np

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


class Gui:

    def __init__(self, start_cal_cb, finish_cal_cb):
        self.server = viser.ViserServer()

        self.cal_started = False

        # Add a Franka to the scene

        urdf = load_robot_description(
            "panda_description",
            load_meshes=True,
            build_scene_graph=True,
            load_collision_meshes=False,
            build_collision_scene_graph=False,
        )

        urdf = yourdfpy.URDF.load(
            "/home/wheitman/wheitman_ws/src/external/FACTR_Teleop/src/factr_teleop/factr_teleop/urdf/factr_teleop_franka.urdf",
            build_scene_graph=True,
            build_collision_scene_graph=False,
            load_meshes=True,
            load_collision_meshes=False,
            filename_handler=partial(
                yourdfpy.filename_handler_magic,
                dir="/home/wheitman/wheitman_ws/src/external/FACTR_Teleop/src/factr_teleop/factr_teleop/urdf/",
            ),
        )

        self.viser_urdf = ViserUrdf(
            self.server,
            urdf_or_path=urdf,
            load_meshes=True,
            load_collision_meshes=False,
            collision_mesh_color_override=(1.0, 0.0, 0.0, 0.5),
        )

        self.cal_button = self.server.gui.add_button("Start Calibration")
        self.explanatory_text = self.server.gui.add_markdown(
            "Click the button to calibrate FACTR's joint angles"
        )
        self.freq_text = self.server.gui.add_markdown("null Hz")
        self.gripper_pos_bar = self.server.gui.add_progress_bar(0.0)
        self.gripper_pos_bar.label = "Gripper"
        self.gripper_pos_bar.hint = "Open/Closed"

        initial_config: list[float] = []
        for joint_name, (
            lower,
            upper,
        ) in self.viser_urdf.get_actuated_joint_limits().items():
            lower = lower if lower is not None else -np.pi
            upper = upper if upper is not None else np.pi
            initial_pos = 0.0 if lower < -0.1 and upper > 0.1 else (lower + upper) / 2.0
            initial_config.append(initial_pos)

        # Start in zero position until told otherwise
        self.viser_urdf.update_cfg(np.zeros(7))

        @self.cal_button.on_click
        def _(_) -> None:
            print("Button clicked!")

            self.cal_started = not self.cal_started

            if self.cal_started:  # Now in calibration mode
                self.explanatory_text.content = "Move FACTR to match the zero configuration shown. Be sure it matches!"
                self.cal_button.label = "Finish Calibration"
                start_cal_cb()

            else:  # Calibration just finished
                self.cal_button.label = "Start Calibration"
                self.explanatory_text.content = (
                    "Click the button to calibrate FACTR's joint angles"
                )
                finish_cal_cb()

    def update(self, config: np.ndarray):
        if self.cal_started:
            self.viser_urdf.update_cfg(np.zeros(7))
            return  # Ignore current config, only show zero config

        self.viser_urdf.update_cfg(config[:7])

        self.gripper_pos_bar.value = config[-1] * (100 / 0.8)  # 0.8 = closed

    def record_freq(self, new_freq):
        self.freq_text.content = f"{int(new_freq)} Hz"


class FactrInterfaceNode(Node):
    """
    Bridges data from Dynamixel motors and robot arms (Franka, Kinova)
    """

    def __init__(self):
        super().__init__("factr_interface")

        self.follower_joint_state = None
        self.follower_joint_angles = np.zeros(8)
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
            JointState, "/factr/joint_state", 10
        )

        self.joint_command_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 1
        )

        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 1)
        self.gripper_client = ActionClient(
            self, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )

        self.factr = Factr()

        self.gui = Gui(self.start_cal, self.finish_cal)

        self.HISTORY_WINDOW_LEN = 1000
        self.spin_times: list[float] = []

        # input("Move FACTR to zero position, then press Enter...")
        # self.factr.cal()

        # self.factr.spring_to_home()

        # self.factr.spring_to(
        #     positions=[0.0 for i in range(8)],
        #     currents=[30, 60, 30, 50, 30, 60, 30, 30],
        #     initialize=True,
        # )

        CONTROLLER_HZ = 500  # TODO WSH: Parameterize
        self.create_timer(1.0 / CONTROLLER_HZ, self.spin_interface)

    def start_cal(self):

        # Cut FACTR torque
        self.factr.cut_torque()

        # Show zero pos FACTR in GUI
        pass

    def finish_cal(self):

        self.factr.cal()

    def set_gripper_pos(self, pos: float, max_effort=100.0):
        """Send an action goal to set the gripper's position

        0.0 = open
        0.8 = close

        Args:
            pos (float): The requested position of the gripper
            max_effort (float, optional): Maximum grip force (unitless). Defaults to 100.0.
        """

        start = time()

        if pos > 0.8:
            pos = 0.8
        elif pos < 0.0:
            pos = 0.0

        command = GripperCommand.Goal()
        command.command.position = pos
        command.command.max_effort = max_effort

        future_ = self.gripper_client.send_goal_async(command)

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

        input("Move to zero position, then press Enter...")

        for motor in self.motors:
            motor.cal()

    def map(self, factr_min, factr_max, follower_min, follower_max, value) -> float:
        factr_range = factr_max - factr_min

        follower_range = follower_max - follower_min

        follower_value = (value - factr_min) / factr_range * follower_range

        # Clip to follower limits
        follower_value = max(follower_value, follower_min)
        follower_value = min(follower_value, follower_max)

        return follower_value

    def spin_interface(self) -> None:

        start = time()

        spring_positions = np.zeros(8)
        spring_positions[-1] = self.follower_joint_angles[-1]
        spring_positions[-2] = self.follower_joint_angles[-2]
        spring_currents = np.asarray([30, 60, 30, 50, 30, 60, 30, 30])

        # self.factr.spring_to(spring_positions, spring_currents)

        self.gui.update(self.factr.pos)

        print(f"Gripper pos: {self.factr.pos[-1]}")

        self.set_gripper_pos(self.factr.pos[-1])

        # Now set the other target joint positions
        command = JointTrajectory()
        command_point = JointTrajectoryPoint()
        command.joint_names = [f"joint_{n}" for n in range(1, 8)]

        command_point.positions = [0.0 for i in range(7)]
        command_point.positions[-1] = self.factr.pos[-2]

        max_joint_angle_error = np.max(
            np.abs(np.asarray(self.follower_joint_angles) - np.asarray(self.factr.pos))
        )

        time_from_start = max_joint_angle_error * 1.0

        command_point.time_from_start.sec = int(np.floor(time_from_start))
        command_point.time_from_start.nanosec = int(
            (time_from_start - np.floor(time_from_start)) * 1e9
        )

        command.points = [command_point]

        # self.joint_command_pub.publish(command)
        self.publish_factr_joint_state()

        self.spin_times.append(time() - start)

        if len(self.spin_times) >= self.HISTORY_WINDOW_LEN:

            freq = 1 / np.mean(self.spin_times)
            self.gui.record_freq(freq)
            self.spin_times.clear()

    def publish_factr_joint_state(self):

        msg = JointState()

        msg.position = self.factr.pos.tolist()
        msg.header.stamp = self.get_clock().now().to_msg()

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
