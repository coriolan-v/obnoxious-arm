#!/usr/bin/env python
#
# *********  Interactive Position Control Example  *********
#
# This example demonstrates how to control a single SC servo's position
# interactively, allowing the user to specify the target position.  It also
# continuously reports the current position.
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
SCS_MOVING_SPEED=100
SCS_MOVING_ACC=20
# Default setting
BAUDRATE                    = 115200
DEVICENAME                  = '/dev/ttyUSB0' # Or your appropriate port

# ... (Other constants like SCS_MINIMUM_POSITION_VALUE, etc. remain the same)

# Initialize PortHandler and PacketHandler
# Initialize PortHandler and PacketHandler (same as before)
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

# Get Servo ID from User
while True:
    try:
        servo_id = int(input("Enter the Servo ID (1-10): "))
        if 1 <= servo_id <= 10:
            break
        else:
            print("Invalid Servo ID. Please enter a number between 1 and 10.")
    except ValueError:
        print("Invalid input. Please enter an integer.")


groupSyncRead = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 11)

while True:
    # Read and print the current position *before* asking for the target
    groupSyncRead = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 11) # Create here
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
    else:
        print(f"[ID:{servo_id:03d}] groupSyncRead getdata failed")
    if scs_error:
        print(packetHandler.getRxPacketError(scs_error))
    groupSyncRead.clearParam()  # Clear after reading
    del groupSyncRead # Important: Delete the object to avoid issues

    try:
        target_position = input("Enter the target position (or 'q' to quit): ")

        # Check for quit command *before* trying to convert to int
        if target_position.lower() == 'q':
            break

        target_position = int(target_position)

        #if SCS_MINIMUM_POSITION_VALUE <= target_position <= SCS_MAXIMUM_POSITION_VALUE:
            # Set the goal position for the specified servo
        scs_addparam_result = packetHandler.SyncWritePosEx(servo_id, target_position, SCS_MOVING_SPEED, SCS_MOVING_ACC)
        if scs_addparam_result != True:
            print(f"[ID:{servo_id:03d}] groupSyncWrite addparam failed")

        # Syncwrite goal position (even for one servo)
        scs_comm_result = packetHandler.groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))

        packetHandler.groupSyncWrite.clearParam()  # Clear after writing

        time.sleep(0.002)  # Small delay

        # Continuously read and print the current position (same as before)
        while True:
            groupSyncRead = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 11) #Create inside the loop
            # ... (rest of the position reading loop remains the same)
            del groupSyncRead #Important: Delete the object to avoid issues


        #else:
        #    print(f"Target position must be between {SCS_MINIMUM_POSITION_VALUE} and {SCS_MAXIMUM_POSITION_VALUE}.")

    except ValueError:
        print("Invalid input. Please enter an integer for the target position or 'q' to quit.")


# Close port
portHandler.closePort()
