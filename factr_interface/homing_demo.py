import os, time
from dynamixel_sdk import *

# Control table addresses for XC330 (Protocol 2.0)
ADDR_TORQUE_ENABLE = 64
ADDR_HOMING_OFFSET = 20
ADDR_PRESENT_POSITION = 132
PROTOCOL_VERSION = 2.0
DXL_ID = 8

# Serial port and baud rate settings
DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 57600

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


def setup_dynamixel():
    """Opens the port and sets the baud rate."""
    if not portHandler.openPort():
        print("Failed to open the port")
        return False
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to change the baudrate")
        return False
    return True


def set_homing_offset_to_zero():
    """
    Reads the current position, calculates the negative offset,
    writes it to the motor, and reboots to apply the change.
    """
    # Disable torque
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)

    # Read the initial present position
    dxl_present_position, result, error = packetHandler.read4ByteTxRx(
        portHandler, DXL_ID, ADDR_PRESENT_POSITION
    )

    if result != COMM_SUCCESS or error != 0:
        print("Failed to read present position.")
        return

    print(f"Initial present position: {dxl_present_position}")

    # Write the new homing offset to the motor
    comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, DXL_ID, ADDR_HOMING_OFFSET, 0
    )

    dxl_present_position, result, error = packetHandler.read4ByteTxRx(
        portHandler, DXL_ID, ADDR_PRESENT_POSITION
    )

    # Calculate the new homing offset as the negative of the current position
    new_homing_offset = -dxl_present_position

    # Write the new homing offset to the motor
    comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, DXL_ID, ADDR_HOMING_OFFSET, new_homing_offset
    )

    if comm_result != COMM_SUCCESS or dxl_error != 0:
        print("Failed to write homing offset.")
        return

    print(f"Homing offset set to {new_homing_offset}.")

    # Reboot the motor
    print("Rebooting the motor...")
    comm_result, dxl_error = packetHandler.reboot(portHandler, DXL_ID)

    if comm_result != COMM_SUCCESS or dxl_error != 0:
        print("Failed to reboot motor.")
        return

    # Wait for the motor to come back online
    time.sleep(2)
    while True:
        try:
            model, ping_result, ping_error = packetHandler.ping(portHandler, DXL_ID)
            if ping_result != COMM_SUCCESS or ping_error != 0:
                print("Pinging motor...")
                time.sleep(0.1)
            else:
                print("Motor is back online.")
                break
        except:
            time.sleep(0.1)

    # Read the present position again to confirm it's zero
    dxl_present_position_new, result, error = packetHandler.read4ByteTxRx(
        portHandler, DXL_ID, ADDR_PRESENT_POSITION
    )

    if result != COMM_SUCCESS or error != 0:
        print("Failed to read present position after reboot.")
        return

    print(f"Present position after reboot: {dxl_present_position_new}")


# Main execution block
if setup_dynamixel():
    set_homing_offset_to_zero()
    # Close the port
    portHandler.closePort()
