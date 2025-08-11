from dynamixel_sdk import PacketHandler, PortHandler, COMM_SUCCESS, GroupSyncRead
import numpy as np
from time import time

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

positions = np.zeros(8)

try:
    while True:
        start = time()

        # Read the position of each motor in the group
        dxl_comm_result = group_sync_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"{packetHandler.getTxRxResult(dxl_comm_result)}")

        for index, dynamixel_id in enumerate(dynamixel_ids):

            positions[index] = group_sync_read.getData(
                dynamixel_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )

        print(positions)
        print(f"Took {time() - start} secs")

except KeyboardInterrupt:
    print(f"User interrupted. Goodbye!")

finally:
    # Close port
    portHandler.closePort()
