import cv2
import dlib
import numpy as np
import time
import sys
import os

# Face detection and tracking
model_file = "deploy.prototxt"  # Replace with your model file
config_file = "res10_300x300_ssd_iter_140000.caffemodel"  # Replace with your config file
net = cv2.dnn.readNetFromCaffe(model_file, config_file)

video_capture = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Or your camera index
if not video_capture.isOpened():
    print("Error: Could not open video capture device.")
    exit()

video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

trackers = []
last_seen = {}
timeout = 2
unique_id = 0
face_ids = {}

def calculate_distance(rect1, rect2):
    center1 = np.array([rect1[0] + rect1[2] / 2, rect1[1] + rect1[3] / 2])
    center2 = np.array([rect2[0] + rect2[2] / 2, rect2[1] + rect2[3] / 2])
    return np.linalg.norm(center1 - center2)

top_height = 250
bottom_height = 700

cv2.namedWindow('Video', cv2.WINDOW_NORMAL)

# Motor control
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

sys.path.append("/home/live/_DEV/obnoxious-arm/Software/SCServo_Python")  # Adjust path if needed
from scservo_sdk import *

SCS_SHOULDER_ID = 1
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB0'  # Or your port
SCS_HOME_POSITION_VALUE = 2000
SCS_MINIMUM_POSITION_VALUE = 1600
SCS_MAXIMUM_POSITION_VALUE = 2700
SCS_MOVING_SPEED = 200  # Reduced speed
SCS_MOVING_ACC = 50

SCS_ELBOW_ID = 2
SCS__ELBOW_HOME_POSITION_VALUE = 1962
SCS_ELBOW_MINIMUM_POSITION_VALUE = 1678
SCS_ELBOW_MAXIMUM_POSITION_VALUE = 2233

portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)
groupSyncRead = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 11) # Initialize sync read

if portHandler.openPort():
    print("Motor port opened successfully")
else:
    print("Failed to open motor port")
    exit()

if portHandler.setBaudRate(BAUDRATE):
    print("Motor baudrate set successfully")
else:
    print("Failed to set motor baudrate")
    portHandler.closePort()
    exit()

min_x = float('inf')
max_x = float('-inf')

scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_SHOULDER_ID, SCS_HOME_POSITION_VALUE, SCS_MOVING_SPEED, SCS_MOVING_ACC)
if scs_comm_result != COMM_SUCCESS:
    print(f"Motor home command failed: {packetHandler.getTxRxResult(scs_comm_result)}")
if scs_error != 0:
    print(f"Motor home error: {packetHandler.getRxPacketError(scs_error)}")
print(f"Motor moved to home position: {SCS_HOME_POSITION_VALUE}")
time.sleep(2) # Give it time to get there
scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ELBOW_ID, SCS_ELBOW_MINIMUM_POSITION_VALUE, SCS_MOVING_SPEED, SCS_MOVING_ACC)
if scs_comm_result != COMM_SUCCESS:
    print(f"Motor home command failed: {packetHandler.getTxRxResult(scs_comm_result)}")
if scs_error != 0:
    print(f"Motor home error: {packetHandler.getRxPacketError(scs_error)}")
print(f"Motor moved to home position: {SCS_HOME_POSITION_VALUE}")

time.sleep(1) # Give it time to get there
# Main loop (Section 3 - next)
while True:
    ret, frame = video_capture.read()

    if not ret:
        print("Error: Could not read frame.")
        break

    # Crop the frame to remove the top and bottom rectangles
    cropped_frame = frame[top_height:frame.shape[0]-bottom_height, :]

    # Get the screen size after the window has been created
    screen_width, screen_height = cv2.getWindowImageRect('Video')[2:4]

    # Calculate the aspect ratio of the cropped frame
    frame_height, frame_width = cropped_frame.shape[:2]
    aspect_ratio = frame_width / frame_height

    # Adjust the dimensions to maintain the aspect ratio
    if screen_width / screen_height > aspect_ratio:
        new_width = int(screen_height * aspect_ratio)
        new_height = screen_height
    else:
        new_width = screen_width
        new_height = int(screen_width / aspect_ratio)

    # Resize the cropped frame to fit the screen size while maintaining the aspect ratio
    resized_frame = cv2.resize(cropped_frame, (new_width, new_height))

    blob = cv2.dnn.blobFromImage(cv2.resize(resized_frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
    net.setInput(blob)
    detections = net.forward()

    faces = []
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:
            box = detections[0, 0, i, 3:7] * np.array([resized_frame.shape[1], resized_frame.shape[0], resized_frame.shape[1], resized_frame.shape[0]])
            (x, y, w, h) = box.astype("int")
            faces.append((x, y, w - x, h - y))

    current_time = time.time()

    new_trackers = []
    for tracker in trackers:
        tracker.update(resized_frame)
        tid = face_ids[tracker]
        if current_time - last_seen[tid] > timeout:
            del last_seen[tid]
            del face_ids[tracker]
        else:
            new_trackers.append(tracker)

    trackers = new_trackers

    tracked_positions = [(tracker.get_position().left(), tracker.get_position().top(), tracker.get_position().width(),
                          tracker.get_position().height()) for tracker in trackers]

    for (x, y, w, h) in faces:
        match_found = False
        for (tx, ty, tw, th) in tracked_positions:
            if calculate_distance((x, y, w, h), (tx, ty, tw, th)) < 50:
                match_found = True
                break
        if not match_found:
            tracker = dlib.correlation_tracker()
            tracker.start_track(resized_frame, dlib.rectangle(x, y, x + w, y + h))
            trackers.append(tracker)
            face_ids[tracker] = unique_id
            last_seen[unique_id] = current_time
            unique_id += 1
            
             # *** NEW: Print coordinates of the first tracked face ***
    if trackers:  # Check if there are any trackers
        first_tracker = trackers[0]
        pos = first_tracker.get_position()
        x = int(pos.left())
        y = int(pos.top())
        w = int(pos.width())
        h = int(pos.height())

        print(f"Coordinates of first tracked face: x={x}, y={y}, w={w}, h={h}")
        
    if trackers:
        for tracker in trackers:
            pos = tracker.get_position()
            x = int(pos.left())
            min_x = min(min_x, x)
            max_x = max(max_x, x)

        # *** NEW: Remap x to 0-1 range for the first face ***
        first_tracker = trackers[0]
        pos = first_tracker.get_position()
        x = int(pos.left())
        
        if max_x - min_x != 0: # Avoid division by zero
            remapped_x = (x - min_x) / (max_x - min_x)
        else:
            remapped_x = 0  # Or handle as you see fit if min and max are equal

        print(f"Original x: {x}, Remapped x (0-1): {remapped_x}")

    for tracker in trackers:
        pos = tracker.get_position()
        x = int(pos.left())
        y = int(pos.top())
        w = int(pos.width())
        h = int(pos.height())
        tid = face_ids[tracker]
        cv2.rectangle(resized_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(resized_frame, f"ID: {tid}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
        # *** NEW: Motor Control with Current Position Reading ***
        # *** NEW: Motor Control with Current Position Reading ***
        center_x = resized_frame.shape[1] // 2
        tolerance = 50

        # 1. Read Current Motor Position
        groupSyncRead.clearParam()  # Clear previous parameters
        scs_addparam_result = groupSyncRead.addParam(SCS_SHOULDER_ID)  # Add the motor ID to the syncread
        if scs_addparam_result != True:
            print(f"[ID:{SCS_SHOULDER_ID}] groupSyncRead addparam failed")

        scs_comm_result = groupSyncRead.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print(f"SyncRead Error: {packetHandler.getTxRxResult(scs_comm_result)}")

        scs_data_result, scs_error = groupSyncRead.isAvailable(SCS_SHOULDER_ID, SMS_STS_PRESENT_POSITION_L, 2)
        if scs_data_result == True:
            current_motor_position = groupSyncRead.getData(SCS_SHOULDER_ID, SMS_STS_PRESENT_POSITION_L, 2)
            print(f"Current Motor Position: {current_motor_position}")

            # 2. Calculate Target Motor Position (using remapped_x)
            error = x - center_x  # Calculate the error
            proportional_gain = 0.25  # Adjust this value - IMPORTANT!
            motor_adjustment = int(proportional_gain * error)

            target_motor_position = current_motor_position + motor_adjustment  # Use current position
            target_motor_position = max(SCS_MINIMUM_POSITION_VALUE, min(SCS_MAXIMUM_POSITION_VALUE, target_motor_position))

            # 3. Move Motor (only if target is different from current)
            if target_motor_position != current_motor_position:
                scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_SHOULDER_ID, target_motor_position, SCS_MOVING_SPEED, SCS_MOVING_ACC)
                if scs_comm_result != COMM_SUCCESS:
                    print(f"Motor command failed: {packetHandler.getTxRxResult(scs_comm_result)}")
                if scs_error != 0:
                    print(f"Motor error: {packetHandler.getRxPacketError(scs_error)}")

                print(f"Target Motor Position: {target_motor_position}")

    else:
        print(f"[ID:{SCS_SHOULDER_ID}] groupSyncRead getdata failed")
        # Handle the case where reading fails (e.g., set current_motor_position to a default or skip motor control)
        # For example, you could add this line:
        # current_motor_position = (SCS_MINIMUM_POSITION_VALUE + SCS_MAXIMUM_POSITION_VALUE) // 2  # Default to center


    cv2.imshow('Video', resized_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

portHandler.closePort()
video_capture.release()
cv2.destroyAllWindows()
