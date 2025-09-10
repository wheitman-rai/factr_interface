# -----------------------------------------------------------------------------
# Description: Bridges data from Dynamixel motors and robot arms (Franka, Kinova)
# Author: Will Heitman
# (c) 2025 RAI Institute
# -----------------------------------------------------------------------------

from functools import partial
from io import TextIOWrapper
import os
import subprocess
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
from dynamixel_sdk.robotis_def import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
)

import numpy as np
import math
import pinocchio as pin
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

# Taken from original paper code
# Converts from Nm to mA
TORQUE_TO_CURRENT_MAPPING = {
    "XC330_T288_T": 1158.73,
    "XM430_W210_T": 1000 / 2.69,
}


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

        # 2. Set the Operating Mode to Current Control (Value 0)
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.id, ADDR_OPERATING_MODE, 0
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

    def set_operating_mode(self, mode: int):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.id, ADDR_TORQUE_ENABLE, mode
        )


class Factr:

    def __init__(
        self,
        ids: list[int] = list(range(1, 9)),
        port: str = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAA0AKR-if00-port0",
        baud: int = 4000000,
    ):

        self.logger = rcutils_logger.RcutilsLogger(name="factr")
        self._torque_enabled = False

        # Establish a connection
        self.port = port
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

        # Describes which motors' axes of rotation are flipped w.r.t. the URDF.
        self.inverted = [False, False, False, True, False, True, False, False]

        # Used to convert torque to mA
        servo_types = [
            "XC330_T288_T",
            "XM430_W210_T",
            "XC330_T288_T",
            "XM430_W210_T",
            "XC330_T288_T",
            "XC330_T288_T",
            "XC330_T288_T",
            "XC330_T288_T",
        ]
        self.torque_to_current_map = np.array(
            [TORQUE_TO_CURRENT_MAPPING[servo] for servo in servo_types]
        )

        self.set_up_motors(ids)

    def check_usb_latency(self):
        """
        TAKEN FROM ORIGINAL PAPER CODE
        checks of the latency timer on ttyUSB of the corresponding port is 1
        if it is not 1, the control loop cannot run at above 200 Hz, which will
        cause extremely undesirable behaviour for the leader arm. If the latency
        timer is not 1, one can set it to 1 as follows:
        echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB{NUM}/latency_timer
        """

        base_path = "/dev/serial/by-id/"
        full_path = os.path.join(base_path, self.port)
        if not os.path.exists(full_path):
            raise Exception(f"Port '{self.port}' does not exist in {base_path}.")
        try:
            resolved_path = os.readlink(full_path)
            actual_device = os.path.basename(resolved_path)
            if not actual_device.startswith("ttyUSB"):
                raise Exception(
                    f"The port '{self.port}' does not correspond to a ttyUSB device. It links to {resolved_path}."
                )
        except Exception as e:
            raise Exception(
                f"Unable to resolve the symbolic link for '{self.port}'. {e}"
            )

        ttyUSBx = actual_device
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

    def set_up_motors(self, ids: list[int]) -> None:
        """
        Initializes and configures the Dynamixel motors for communication.
        Opens the serial port, sets the baud rate, and adds each motor ID to the sync read group.
        Logs errors if connection or configuration fails.
        """

        # Instantiate 8 Dynamixels
        self.dynamixels: list[Dynamixel] = []
        for id, inverted in zip(ids, self.inverted):
            self.dynamixels.append(
                Dynamixel(id, self.packet_handler, self.port_handler, inverted)
            )

            # Add the motor to the Sync Read Group
            if not self.sync_read_group.addParam(id):
                self.logger.error(f"Failed to add motor {id} to the sync read group")

        self.check_usb_latency()

        self.set_operating_mode(0)  # Current mode

    def set_operating_mode(self, mode: int):

        self.disable_torque()

        for dynamixel in self.dynamixels:
            dynamixel.set_operating_mode(mode)

        self.enable_torque()

    def disable_torque(self):
        # TODO WSH: Optimize with write sync group
        for dynamixel in self.dynamixels:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, dynamixel.id, ADDR_TORQUE_ENABLE, 0
            )

        self._torque_enabled = False

    def enable_torque(self):
        # TODO WSH: Optimize with write sync group
        for dynamixel in self.dynamixels:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, dynamixel.id, ADDR_TORQUE_ENABLE, 1
            )
        self._torque_enabled = True

    def set_torques(self, torques: np.ndarray) -> None:
        """Sets the torque of each motor in the chain. Maps to currents.

        Args:
            torques (np.ndarray): Torques WITHOUT accounting for inversion. Direction inversion is handled here!
        """

        if len(torques) != len(self.dynamixels):
            self.logger.error(
                f"Got {len(torques)} torques but FACTR has {len(self.dynamixels)} motors. Torques won't be set."
            )
            return
        currents = self.torque_to_current_map * torques
        self.set_currents(currents)

    def set_currents(self, currents: np.ndarray):

        if len(currents) != len(self.dynamixels):
            raise ValueError("The length of currents must match the number of servos")
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled to set currents")

        # Clip them to a reasonable range
        if np.max(currents) > 900:
            self.logger.warning(
                f"Currents for motor {np.argmax(currents) + 1} was {np.max(currents)}"
            )
        if np.min(currents) < -900:
            self.logger.warning(
                f"Currents for motor {np.argmax(currents) + 1} was {np.min(currents)}"
            )

        currents = np.clip(currents, -900, 900)

        currents = np.clip(currents, -900, 900)
        for dynamixel, current in zip(self.dynamixels, currents):
            current_value = int(current)

            param_goal_current = [DXL_LOBYTE(current_value), DXL_HIBYTE(current_value)]

            if not self.sync_write_current.addParam(dynamixel.id, param_goal_current):
                raise RuntimeError(
                    f"Failed to set current for Dynamixel with ID {dynamixel.id}"
                )
        dxl_comm_result = self.sync_write_current.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError("Failed to syncwrite goal current")
        self.sync_write_current.clearParam()

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

        positions: list[float] = []

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
                (
                    "serial_port",
                    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAA0AKR-if00-port0",
                    ParameterDescriptor(),
                ),
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

        urdf_path = "/home/wheitman/wheitman_ws/src/factr_interface/urdf/factr_teleop_franka.urdf"

        # 1. Load the robot model from the URDF file
        self.model, _, _ = pin.buildModelsFromUrdf(
            filename=urdf_path,
            package_dirs="/home/wheitman/wheitman_ws/src/factr_interface/urdf",
        )
        self.model_data = (
            self.model.createData()
        )  # The data structure for the dynamics model

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

    def get_gravity_torques(self):

        # Define the robot's state: joint positions, velocities, and accelerations
        # For gravity compensation, velocities and accelerations are zero.
        q = self.factr.pos[:-1]  # Ignore the gripper position
        # q = np.zeros(self.model.nv)  # Joint configuration (e.g., 45 and 90 degrees)
        # q[1] = np.pi / 2
        v = np.zeros(self.model.nv)  # Zero joint velocities
        a = np.zeros(self.model.nv)  # Zero joint accelerations

        # Calculate the required inverse dynamics torques using RNEA
        # The pin.rnea function computes the full inverse dynamics equation:
        # tau = M(q) * a + C(q, v) * v + G(q)
        # Since 'v' and 'a' are zero, the result is simply the gravity vector G(q).
        torques = pin.rnea(self.model, self.model_data, q, v, a)

        return torques

    def start_cal(self):

        # Cut FACTR torque
        self.factr.disable_torque()

        # Show zero pos FACTR in GUI
        pass

    def finish_cal(self):

        self.factr.cal()

    def set_follower_gripper_pos(self, pos: float, max_effort=100.0):
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

        self.gui.update(self.factr.pos)

        # GRAVITY COMP & FORCE FEEDBACK
        GRAVITY_STRENGTH = 0.5
        tau_g = self.get_gravity_torques() * GRAVITY_STRENGTH
        print(tau_g)

        gripper_torque = 0.0

        self.factr.set_torques(np.concatenate([tau_g, [gripper_torque]], axis=0))

        # PUBLISH COMMAND TO FOLLOWER
        self.set_follower_gripper_pos(self.factr.pos[-1])
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

        # PROFILING/TIMING CALCULATION
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

    try:

        rclpy.spin(node)

    finally:

        logger = rcutils_logger.RcutilsLogger(name="factr")
        for i in range(3):
            logger.info(f"Disabling torque in {3-i}")
            sleep(1.0)
        logger.info("Disabling torque. Goodbye.")
        node.factr.disable_torque()

        # Destroy the node explicitly
        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
