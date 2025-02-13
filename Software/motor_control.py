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

sys.path.append("/home/live/Documents/GitHub/obnoxious-arm/Software/SCServo_Python")  # Adjust path if needed
from scservo_sdk import *

BAUDRATE = 115200

DEVICENAME = '/dev/ttyUSB0'  # Or your port

SCS_SHOULDER_ID = 1
SCS_SHOULDER_HOME_POSITION_VALUE = 2000
SCS_SHOULDER_MINIMUM_POSITION_VALUE = 1000
SCS_SHOULDER_MAXIMUM_POSITION_VALUE = 2700
SCS_SHOULDER_MOVING_SPEED = 200
SCS_SHOULDER_MOVING_ACC = 50

SCS_ELBOW_ID = 2
SCS_ELBOW_HOME_POSITION_VALUE = 1962
SCS_ELBOW_MINIMUM_POSITION_VALUE = 1678
SCS_ELBOW_MAXIMUM_POSITION_VALUE = 2250
SCS_ELBOW_MOVING_SPEED = 200
SCS_ELBOW_MOVING_ACC = 50

class MotorController:
    def __init__(self):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = sms_sts(self.portHandler)
        self.groupSyncRead = GroupSyncRead(self.packetHandler, SMS_STS_PRESENT_POSITION_L, 11)  # Initialize sync read
        self.groupSyncReadElbow = GroupSyncRead(self.packetHandler, SMS_STS_PRESENT_POSITION_L, 11)  # Initialize sync read for elbow
        self.open_port()
        self.set_baudrate()
        self.go_home()

    def open_port(self):
        if self.portHandler.openPort():
            print("Motor port opened successfully")
        else:
            print("Failed to open motor port")
            exit()

    def set_baudrate(self):
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Motor baudrate set successfully")
        else:
            print("Failed to set motor baudrate")
            self.portHandler.closePort()
            exit()

    def go_home(self):
        self.move_motor(SCS_SHOULDER_ID, SCS_SHOULDER_HOME_POSITION_VALUE)
        self.move_motor(SCS_ELBOW_ID, SCS_ELBOW_HOME_POSITION_VALUE)
        time.sleep(2)  # Give it time to get there

    def move_motor(self, motor_id, position):
    
        SCS_MOVING_SPEED = 200  # Reduced speed
        SCS_MOVING_ACC = 30

        if motor_id == SCS_SHOULDER_ID:
            SCS_MOVING_ACC = SCS_SHOULDER_MOVING_ACC
            SCS_MOVING_SPEED = SCS_SHOULDER_MOVING_SPEED
        else:
            SCS_MOVING_ACC = SCS_ELBOW_MOVING_ACC
            SCS_MOVING_SPEED = SCS_ELBOW_MOVING_SPEED
        scs_comm_result, scs_error = self.packetHandler.WritePosEx(motor_id, position, SCS_MOVING_SPEED, SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS:
            print(f"Motor command failed: {self.packetHandler.getTxRxResult(scs_comm_result)}")
        if scs_error != 0:
            print(f"Motor error: {self.packetHandler.getRxPacketError(scs_error)}")

    def control_shoulder_motor(self, x, center_x, tolerance):
        self.groupSyncRead.clearParam()
        scs_addparam_result = self.groupSyncRead.addParam(SCS_SHOULDER_ID)
        if scs_addparam_result != True:
            print(f"[ID:{SCS_SHOULDER_ID}] groupSyncRead addparam failed")

        scs_comm_result = self.groupSyncRead.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print(f"SyncRead Error: {self.packetHandler.getTxRxResult(scs_comm_result)}")

        scs_data_result, scs_error = self.groupSyncRead.isAvailable(SCS_SHOULDER_ID, SMS_STS_PRESENT_POSITION_L, 2)
        if scs_data_result == True:
            current_shoulder_position = self.groupSyncRead.getData(SCS_SHOULDER_ID, SMS_STS_PRESENT_POSITION_L, 2)
            #print(f"Current Shoulder Position: {current_shoulder_position}") #optional debug print

            error = x - center_x
            proportional_gain = 0.20  # Adjust this value - IMPORTANT!
            motor_adjustment = int(proportional_gain * error)

            target_shoulder_position = current_shoulder_position + motor_adjustment
            target_shoulder_position = max(SCS_SHOULDER_MINIMUM_POSITION_VALUE, min(SCS_SHOULDER_MAXIMUM_POSITION_VALUE, target_shoulder_position))

            if target_shoulder_position != current_shoulder_position:
                self.move_motor(SCS_SHOULDER_ID, target_shoulder_position)
                #print(f"Target Shoulder Position: {target_shoulder_position}") #optional debug print

        else:
            #print(f"[ID:{SCS_SHOULDER_ID}] groupSyncRead getdata failed") #optional debug print
            current_shoulder_position = (SCS_SHOULDER_MINIMUM_POSITION_VALUE + SCS_SHOULDER_MAXIMUM_POSITION_VALUE) // 2  # Default to center

    def control_elbow_motor(self, y, center_y, tolerance_y):
        self.groupSyncReadElbow.clearParam()
        scs_addparam_result = self.groupSyncReadElbow.addParam(SCS_ELBOW_ID)
        if scs_addparam_result != True:
            print(f"[ID:{SCS_ELBOW_ID}] groupSyncRead addparam failed")

        scs_comm_result = self.groupSyncReadElbow.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print(f"SyncRead Error: {self.packetHandler.getTxRxResult(scs_comm_result)}")

        scs_data_result, scs_error = self.groupSyncReadElbow.isAvailable(SCS_ELBOW_ID, SMS_STS_PRESENT_POSITION_L, 2)

        if scs_data_result == True:
            current_elbow_position = self.groupSyncReadElbow.getData(SCS_ELBOW_ID, SMS_STS_PRESENT_POSITION_L, 2)
            #print(f"Current Elbow Position: {current_elbow_position}") #optional debug print

            error_y = y - center_y
            proportional_gain_y = 0.25  # Increased gain for y-axis
            motor_adjustment_y = int(proportional_gain_y * error_y)

            target_elbow_position = current_elbow_position - motor_adjustment_y
            target_elbow_position = max(SCS_ELBOW_MINIMUM_POSITION_VALUE, min(SCS_ELBOW_MAXIMUM_POSITION_VALUE, target_elbow_position))

            if abs(error_y) > tolerance_y:
                self.move_motor(SCS_ELBOW_ID, target_elbow_position)
                #print(f"Target Elbow Position: {target_elbow_position}") #optional debug print
                #print(f"Current Elbow Position: {current_elbow_position}") #optional debug print
            #else:
                #print("Error within tolerance - motor not moved") #optional debug print

        else:
            #print(f"[ID:{SCS_ELBOW_ID}] groupSyncRead getdata failed") #optional debug print
            current_elbow_position = (SCS_ELBOW_MINIMUM_POSITION_VALUE + SCS_ELBOW_MAXIMUM_POSITION_VALUE) // 2
            #print("Elbow groupSyncRead failed, setting default position") #optional debug print

    def close_port(self):
        self.portHandler.closePort()
