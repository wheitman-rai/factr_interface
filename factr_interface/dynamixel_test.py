from dynamixel_sdk import PacketHandler, PortHandler, COMM_SUCCESS
import numpy as np

# FACTR and GELLO expect IDs from 1-8
dynamixel_ids = list(range(1, 9))

# See https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#control-table-of-ram-area
ADDR_PRESENT_POSITION = 132

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler('/dev/ttyUSB0')

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(protocol_version=2.0)

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

positions = np.zeros(8)

try:
    while True:
        for index, dynamixel_id in enumerate(dynamixel_ids):

            # Get current position
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dynamixel_id, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                # print(f"Motor {dynamixel_id} has position {dxl_present_position}")
                positions[index] = dxl_present_position

        print(positions)

except KeyboardInterrupt:
    print(f"User interrupted. Goodbye!")

finally:
    # Close port
    portHandler.closePort()