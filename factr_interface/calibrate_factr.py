from dynamixel_sdk import PacketHandler, PortHandler, COMM_SUCCESS, GroupSyncRead
import numpy as np
from time import time
from tqdm import trange

# FACTR and GELLO expect IDs from 1-8
dynamixel_ids = list(range(1, 9))

# See https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#control-table-of-ram-area
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4  # 4 bytes for position data

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler("/dev/ttyUSB0")

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(protocol_version=2.0)

group_sync_read = GroupSyncRead(
    portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()


# Set port baudrate
if portHandler.setBaudRate(57600):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# Add all motors to the read group
for dynamixel_id in dynamixel_ids:
    if not group_sync_read.addParam(dynamixel_id):
        print(f"Failed to add motor {dynamixel_id} to the sync read group")
        exit()


def get_positions(degrees=False, offset=False):
    positions = []
    dxl_comm_result = group_sync_read.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print(f"{packetHandler.getTxRxResult(dxl_comm_result)}")

    limp_positions = [362.2, -140.2, -272.0, 180.4, 88.9, 104.2, 180.9, 185.0]

    for idx, dynamixel_id in enumerate(dynamixel_ids):

        pos = group_sync_read.getData(
            dynamixel_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
        )

        # Handle signed integers from Dynamixel
        if pos > 2**31:
            pos -= 2**32

        if degrees:
            pos *= 0.087891  # Dynamixel motor's typical resolution, in deg/tick

        if offset:
            # Offset the position relative to zero position
            # (FACTR pointing straight up)
            pos -= limp_positions[idx]

        positions.append(pos)

    return positions


print("This script will help you calibrate FACTR in just a few steps.")

joint_ranges = []

try:

    while True:
        positions = get_positions(degrees=True, offset=True)

        for idx, position in enumerate(positions):
            print(f"{idx+1}: {position}")

        print("-------")

finally:
    # Close port
    portHandler.closePort()
