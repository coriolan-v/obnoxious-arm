#!/usr/bin/env python
#
# *********  Immediate Position Control Example  *********
#
# This example demonstrates how to move a single SC servo to a specific
# position immediately and continuously report the current position.
#

import sys
import os
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

try:
    sys.path.append("/home/live/Documents/GitHub/obnoxious-arm/Software/SCServo_Python")  # Adjust path if needed
    from scservo_sdk import *
except ImportError:
    print("Error: Could not import scservo_sdk. Make sure it's installed and the path is correct.")
    exit()

# Control table address
# ... (Control table addresses remain the same as your original code)
SCS_MOVING_SPEED = 100  # Adjust as needed
SCS_MOVING_ACC = 20    # Adjust as needed

# Default setting
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB0'  # Or your appropriate port

# ... (Other constants like SCS_MINIMUM_POSITION_VALUE, etc. remain the same)

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

servo_id = 2  # Servo ID 2
target_position = 1550

# Move servo to target position immediately
scs_addparam_result = packetHandler.SyncWritePosEx(servo_id, target_position, SCS_MOVING_SPEED, SCS_MOVING_ACC)
if scs_addparam_result != True:
    print(f"[ID:{servo_id:03d}] SyncWritePosEx addparam failed")

scs_comm_result = packetHandler.groupSyncWrite.txPacket()
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))

packetHandler.groupSyncWrite.clearParam()

# Continuously read and print the current position
while True:
    groupSyncRead = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 11)
    scs_addparam_result = groupSyncRead.addParam(servo_id)
    if scs_addparam_result != True:
        print(f"[ID:{servo_id:03d}] groupSyncRead addparam failed")
    scs_comm_result = groupSyncRead.txRxPacket()
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))

    scs_data_result, scs_error = groupSyncRead.isAvailable(servo_id, SMS_STS_PRESENT_POSITION_L, 2)
    if scs_data_result == True:
        current_position = groupSyncRead.getData(servo_id, SMS_STS_PRESENT_POSITION_L, 2)
        current_speed = groupSyncRead.getData(servo_id, SMS_STS_PRESENT_SPEED_L, 2)
        current_moving = groupSyncRead.getData(servo_id, SMS_STS_MOVING, 1)


        print(f"[ID:{servo_id:03d}] Current Position: {current_position} Speed:{packetHandler.scs_tohost(current_speed, 15)}")
        if current_moving == 0: # Check if motor stopped moving
            break # Exit loop if motor stopped moving
    else:
        print(f"[ID:{servo_id:03d}] groupSyncRead getdata failed")
    if scs_error:
        print(packetHandler.getRxPacketError(scs_error))
    groupSyncRead.clearParam()
    del groupSyncRead #Important: Delete the object to avoid issues
    time.sleep(0.01) # Small delay for reading



# Close port (you might want to keep this open if you have other operations)
#portHandler.closePort()  # Commented out to potentially keep the port open
