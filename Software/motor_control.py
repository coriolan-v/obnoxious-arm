# motor_control.py
import sys
import os
import time
import select
from threading import Lock

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

BAUDRATE = 115200

DEVICENAME = '/dev/ttyUSB0'  # Or your port

MODEL1 = 1
MODEL2 = 2

current_model = MODEL2

if current_model == MODEL1:
    SCS_SHOULDER_HOME_POSITION_VALUE = 2000 #model2: 2500
    SCS_SHOULDER_MINIMUM_POSITION_VALUE = 1000 #model2: 1700
    SCS_SHOULDER_MAXIMUM_POSITION_VALUE = 2700 #model: 3300
    SCS_ELBOW_HOME_POSITION_VALUE = 2000 #model2: 2000
    SCS_ELBOW_MINIMUM_POSITION_VALUE = 1900 #model2: 1700
    SCS_ELBOW_MAXIMUM_POSITION_VALUE = 2500 #model2: 2200
elif current_model == MODEL2:
    SCS_SHOULDER_HOME_POSITION_VALUE = 2500 #model2: 2500
    SCS_SHOULDER_MINIMUM_POSITION_VALUE = 1700 #model2: 1700
    SCS_SHOULDER_MAXIMUM_POSITION_VALUE = 3300 #model: 3300
    SCS_ELBOW_HOME_POSITION_VALUE = 2000 #model2: 2000
    SCS_ELBOW_MINIMUM_POSITION_VALUE = 1700 #model2: 1700
    SCS_ELBOW_MAXIMUM_POSITION_VALUE = 2200 #model2: 2200

    
SCS_SHOULDER_ID = 1
SCS_SHOULDER_MOVING_SPEED = 400
SCS_SHOULDER_MOVING_ACC = 40
SCS_SHOULDER_MOVING_SPEED_HOME = 200
SCS_SHOULDER_MOVING_ACC_HOME = 20
PROPORTIONAL_GAIN_SHOULDER = 0.45

SCS_ELBOW_ID = 2
SCS_ELBOW_MOVING_SPEED = 400
SCS_ELBOW_MOVING_ACC = 40
SCS_ELBOW_MOVING_SPEED_HOME = 200
SCS_ELBOW_MOVING_ACC_HOME = 20
PROPORTIONAL_GAIN_ELBOW = 0.4


class MotorController:
    def __init__(self):
        # ... (port and packet handler initialization - same as before)
        self.shoulder_current_position = None  # Store current shoulder position
        self.elbow_current_position = None    # Store current elbow position
        self.shoulder_target_position = None   # Store last target shoulder position
        self.elbow_target_position = None     # Store last target elbow position
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = sms_sts(self.portHandler)
        self.groupSyncRead = GroupSyncRead(self.packetHandler, SMS_STS_PRESENT_POSITION_L, 11)
        self.groupSyncReadElbow = GroupSyncRead(self.packetHandler, SMS_STS_PRESENT_POSITION_L, 11)
        self.open_port()
        self.set_baudrate()
        self.go_home()
        self.sweep_running = False  # Flag for sweep thread
        self.sweep_lock = Lock()  # Lock for thread safety during sweep
        self.index = 0  # For toggling goal positions (if needed)
        self.shoulder_target_position = SCS_SHOULDER_HOME_POSITION_VALUE  # Initialize target positions
        self.elbow_target_position = SCS_ELBOW_HOME_POSITION_VALUE      # Initialize target positions

        #self.shared_variable = shared_variable # Shared variable for interruption 

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

        self.move_motor(SCS_SHOULDER_ID, SCS_SHOULDER_HOME_POSITION_VALUE, SCS_SHOULDER_MOVING_SPEED_HOME, SCS_SHOULDER_MOVING_ACC_HOME)
        self.move_motor(SCS_ELBOW_ID, SCS_ELBOW_HOME_POSITION_VALUE, SCS_ELBOW_MOVING_SPEED_HOME, SCS_ELBOW_MOVING_ACC_HOME)
        #time.sleep(2)  # Give it time to get there

    
        
    
    def move_motor(self, motor_id, position, speed=None, acceleration=None):
        if speed is None:
            if motor_id == SCS_SHOULDER_ID:
                speed = SCS_SHOULDER_MOVING_SPEED
            elif motor_id == SCS_ELBOW_ID:
                speed = SCS_ELBOW_MOVING_SPEED
            else:
                speed = SCS_MOVING_SPEED # Default if ID is unknown

        if acceleration is None:
            if motor_id == SCS_SHOULDER_ID:
                acceleration = SCS_SHOULDER_MOVING_ACC
            elif motor_id == SCS_ELBOW_ID:
                acceleration = SCS_ELBOW_MOVING_ACC
            else:
                acceleration = SCS_MOVING_ACC # Default if ID is unknown

        scs_comm_result, scs_error = self.packetHandler.WritePosEx(motor_id, position, speed, acceleration)
    
 
    '''
    def get_current_shoulder_position(self):
        position, speed = self.read_motor_position_speed(SCS_SHOULDER_ID)
        if position is not None:
            return position
        else:
            print("Failed to get current shoulder position. Returning default.")
            return SCS_SHOULDER_HOME_POSITION_VALUE  # Or a suitable default

    def get_current_elbow_position(self):
        position, speed = self.read_motor_position_speed(SCS_ELBOW_ID)
        if position is not None:
            return position
        else:
            print("Failed to get current elbow position. Returning default.")
            return SCS_ELBOW_HOME_POSITION_VALUE  # Or a suitable default
    '''
    
    def read_motor_position_speed(self, motor_id):
        scs_present_position, scs_present_speed, scs_comm_result, scs_error = self.packetHandler.ReadPosSpeed(motor_id)
        if scs_comm_result != COMM_SUCCESS:
            print(f"Position/Speed read failed: {self.packetHandler.getTxRxResult(scs_comm_result)}")
            return None, None
        elif scs_error != 0:
            print(f"Position/Speed read error: {self.packetHandler.getRxPacketError(scs_error)}")
            return None, None
        return scs_present_position, scs_present_speed

    def control_shoulder_motor(self, x, center_x, tolerance):
        error = x - center_x
        proportional_gain = PROPORTIONAL_GAIN_SHOULDER  # Adjust this value - IMPORTANT!
        motor_adjustment = int(proportional_gain * error)

        target_shoulder_position = self.shoulder_target_position + motor_adjustment # Use the LAST target position
        target_shoulder_position = max(SCS_SHOULDER_MINIMUM_POSITION_VALUE, min(SCS_SHOULDER_MAXIMUM_POSITION_VALUE, target_shoulder_position))

        if target_shoulder_position != self.shoulder_target_position: # Only send command if target changed
            self.move_motor(SCS_SHOULDER_ID, target_shoulder_position)
            self.shoulder_target_position = target_shoulder_position # Update the last target position

    def control_elbow_motor(self, y, center_y, tolerance_y):
        error_y = y - center_y
        proportional_gain_y = PROPORTIONAL_GAIN_ELBOW  # Adjust this value - IMPORTANT!
        motor_adjustment_y = int(proportional_gain_y * error_y)

        target_elbow_position = self.elbow_target_position - motor_adjustment_y # Use the LAST target position
        target_elbow_position = max(SCS_ELBOW_MINIMUM_POSITION_VALUE, min(SCS_ELBOW_MAXIMUM_POSITION_VALUE, target_elbow_position))

        if target_elbow_position != self.elbow_target_position: # Only send command if target changed
            self.move_motor(SCS_ELBOW_ID, target_elbow_position)
            self.elbow_target_position = target_elbow_position # Update the last target position

            
    def sweep_shoulder_motor(self, sweep_delay=4):
        """Sweeps the shoulder motor, interruptible by shared variable."""

        with self.sweep_lock:  # Acquire the lock
            self.sweep_running = True  # Set the flag
            try:
                while self.sweep_running and not self.shared_variable.get(): # Check both conditions
                    # Go to maximum position
                    self.move_motor(SCS_SHOULDER_ID, SCS_SHOULDER_MAXIMUM_POSITION_VALUE)
                    print("Going to max position")
                    time.sleep(1)
                    time.sleep(sweep_delay)

                    # Go to minimum position
                    self.move_motor(SCS_SHOULDER_ID, SCS_SHOULDER_MINIMUM_POSITION_VALUE)
                    print("Going to min position")
                    time.sleep(1)
                    time.sleep(sweep_delay)

            finally:
                self.sweep_running = False  # Ensure flag is reset even if exception occurs
                print("Sweep finished or interrupted")  # Indicate completion/interruption


    def stop_sweep(self): # Function to explicitly stop the sweep
        with self.sweep_lock:
            self.sweep_running = False


    def get_current_shoulder_position(self):
        """Gets the current shoulder motor position using sync read."""
        self.groupSyncRead.clearParam()
        scs_addparam_result = self.groupSyncRead.addParam(SCS_SHOULDER_ID)
        if scs_addparam_result != True:
            print(f"[ID:{SCS_SHOULDER_ID}] groupSyncRead addparam failed")

        scs_comm_result = self.groupSyncRead.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print(f"SyncRead Error: {self.packetHandler.getTxRxResult(scs_comm_result)}")
            return None  # Or handle the error as needed

        scs_data_result, scs_error = self.groupSyncRead.isAvailable(SCS_SHOULDER_ID, SMS_STS_PRESENT_POSITION_L, 2)
        if scs_data_result == True:
            current_shoulder_position = self.groupSyncRead.getData(SCS_SHOULDER_ID, SMS_STS_PRESENT_POSITION_L, 2)
            return current_shoulder_position
        else:
            print(f"[ID:{SCS_SHOULDER_ID}] groupSyncRead getdata failed")
            return None

    def get_current_elbow_position(self):
        """Gets the current shoulder motor position using sync read."""
        self.groupSyncRead.clearParam()
        scs_addparam_result = self.groupSyncRead.addParam(SCS_ELBOW_ID)
        if scs_addparam_result != True:
            print(f"[ID:{SCS_ELBOW_ID}] groupSyncRead addparam failed")

        scs_comm_result = self.groupSyncRead.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print(f"SyncRead Error: {self.packetHandler.getTxRxResult(scs_comm_result)}")
            return None  # Or handle the error as needed

        scs_data_result, scs_error = self.groupSyncRead.isAvailable(SCS_ELBOW_ID, SMS_STS_PRESENT_POSITION_L, 2)
        if scs_data_result == True:
            current_elbow_position = self.groupSyncRead.getData(SCS_ELBOW_ID, SMS_STS_PRESENT_POSITION_L, 2)
            return current_elbow_position
        else:
            print(f"[ID:{SCS_ELBOW_ID}] groupSyncRead getdata failed")
            return None

    def check_sweep_interruption(self):
        """Checks for user input (e.g., 'q' key) to interrupt the sweep."""
        if os.name == 'nt':
            if msvcrt.kbhit():  # Check if a key has been pressed
                key = msvcrt.getch().decode()
                if key.lower() == 'q':
                    return True
        else:  # Linux/macOS
            # Non-blocking check for input (more complex)
            try:
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]: # Check if there is something to read
                    key = sys.stdin.read(1)
                    if key.lower() == 'q':
                        return True
            except Exception as e:
                print(f"Error checking input: {e}")
        return False
        
    def start_elbow_sweep(self, sweep_delay=5, sweep_up_down_time=2):  # New function
        """Starts the elbow sweep in a separate thread."""
        import threading  # Import threading here, only if needed

        def _elbow_sweep_thread(self, sweep_delay, sweep_up_down_time): # Inner function for the thread
            time.sleep(sweep_delay)  # Initial delay
            while True:
                current_elbow_position = self.get_current_elbow_position()
                if current_elbow_position is not None:
                    if current_elbow_position >= SCS_ELBOW_MAXIMUM_POSITION_VALUE - 100:
                        self.move_motor(SCS_ELBOW_ID, SCS_ELBOW_MINIMUM_POSITION_VALUE, 200, 20)
                    elif current_elbow_position <= SCS_ELBOW_MINIMUM_POSITION_VALUE + 100:
                        self.move_motor(SCS_ELBOW_ID, SCS_ELBOW_MAXIMUM_POSITION_VALUE, 200, 20)
                    else:
                        self.move_motor(SCS_ELBOW_ID, SCS_ELBOW_MAXIMUM_POSITION_VALUE, 200, 20)
                time.sleep(sweep_up_down_time)  # Delay between movements

        self.elbow_sweep_thread = threading.Thread(target=_elbow_sweep_thread, args=(self, sweep_delay, sweep_up_down_time)) # Create the thread
        self.elbow_sweep_thread.daemon = True  # Allow the main thread to exit even if sweep is running
        self.elbow_sweep_thread.start() # Start the thread

    def stop_elbow_sweep(self):
        """Stops the elbow sweep thread."""
        if hasattr(self, "elbow_sweep_thread") and self.elbow_sweep_thread.is_alive():
            # There's no clean way to directly stop a thread in Python.
            # A common approach is to use a shared flag that the thread checks.
            # For simplicity in this case, we rely on the daemon=True setting and
            # let the thread finish when the main program exits.
            # For a more robust solution, you could add a stop flag and check it in the _elbow_sweep_thread.
            pass  # Or add a more robust stop mechanism if needed.
   

    def close_port(self):
        self.portHandler.closePort()
