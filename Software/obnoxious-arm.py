#/home/live/Documents/GitHub/obnoxious-arm/Software/haar+motor.py
import os
import cv2
import time
import numpy as np  # Import numpy
from motor_control import MotorController  # Import your motor control class
import subprocess  # For piping to v4l2loopback

FPS_DISPLAY_INTERVAL = 10  # Display FPS every 10 frames
frame_count = 0  # Initialize frame counter
CAMERA_WIDTH = 320  # Reduced width for faster processing
CAMERA_HEIGHT = 240  # Reduced height for faster processing
# Open the default camera (video0)

# Check if /dev/video0 exists, try again every 5 seconds
print("checking /dev/video0...")
while not os.path.exists("/dev/video0"):
    print("/dev/video0 not found. Retrying in 5 seconds...")
    time.sleep(5)
    
cap = cv2.VideoCapture("/dev/video0")
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

# Load a pre-trained face detection model (Haar cascade - faster)
face_cascade = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml") # Adjust path if needed

tracked_faces = {}  # Dictionary to store tracked face data (x, y, w, h, last_seen_frame)
next_face_id = 0  # Initialize face ID counter
ID_PERSISTENCE_FRAMES = 100  # Number of frames before a lost face ID is reassigned

# Motor position mapping ranges (adjust these to your motor's limits)
SHOULDER_MOTOR_MIN = 1000  # Replace with your actual minimum value
SHOULDER_MOTOR_MAX = 2700  # Replace with your actual maximum value
ELBOW_MOTOR_MIN = 1678     # Replace with your actual minimum value
ELBOW_MOTOR_MAX = 2250   # Replace with your actual maximum value

SHOULDER_OFFSET = 50  # Offset to the right (positive) or left (negative)
ELBOW_OFFSET = 0     # Offset down (positive) or up (negative)

# Initialize motor controller
motor_controller = MotorController()

# For SSD (Slower, More Accurate - requires .prototxt and .caffemodel files)
# proto = "deploy.prototxt"  # Path to the .prototxt file
# modelFile = "mobilenet_ssd_caffe.caffemodel" # Path to the .caffemodel file
# net = cv2.dnn.readNetFromCaffe(proto, modelFile)
# if net.empty():
#     print("Error: Could not load SSD model.")
#     exit()

face_id = 0  # Initialize face ID counter

DISPLAY_SCALE = 2  # Double the size of the preview window (adjust as needed)
scale_factor = 0.5  # Adjust as needed#

'''
# V4L2 loopback device path
V4L2_DEVICE = "/dev/video1"

# Open a pipe to the v4l2loopback device using ffmpeg
command = [
    'ffmpeg', '-y',
    '-f', 'rawvideo',
    '-vcodec','rawvideo',
    '-s', f'{int(CAMERA_WIDTH*scale_factor)}x{int(CAMERA_HEIGHT*scale_factor)}',  # Now defined!
    '-pix_fmt', 'bgr24',
    '-i', '-',
    '-f', 'v4l2',
    V4L2_DEVICE
]
pipe = subprocess.Popen(command, stdin=subprocess.PIPE)
'''

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break


    # 1. Resize FIRST (if needed)
   
    resized_frame = cv2.resize(frame, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_LINEAR)

    # 2. Rotate (Efficiently)
    rotated_frame = cv2.transpose(resized_frame)
    rotated_frame = cv2.flip(rotated_frame, flipCode=1)

    # Efficient 90-degree rotation (transpose and flip)
    rotated_frame = cv2.transpose(resized_frame)
    rotated_frame = cv2.flip(rotated_frame, flipCode=1) # Flip horizontally after transpose for CCW 90 degree rotation.

    start_time = time.time()

    # Face detection (same as before)
    faces = face_cascade.detectMultiScale(
    rotated_frame,
    scaleFactor=1.07,      # Adjust this value (1.05-1.3)
    minNeighbors=4,      # Adjust this value (3-6 or higher)
    flags=cv2.CASCADE_SCALE_IMAGE,
    minSize=(20, 20)     # Keep this or adjust as needed
    )
    end_time = time.time()
    fps = 1 / (end_time - start_time)

    # Update tracked faces
    new_tracked_faces = {}  # Store updated face information
    for x, y, w, h in faces:
        best_match_id = None
        min_distance = float('inf')  # Initialize minimum distance to infinity

        for face_id, face_data in tracked_faces.items():
            last_x, last_y, _, _ = face_data['rect']
            distance = np.sqrt((x - last_x)**2 + (y - last_y)**2)  # Calculate the distance between the centers of the rects
            if distance < min_distance:
                min_distance = distance
                best_match_id = face_id

        if best_match_id is not None and min_distance < 50:
            # Update existing face
            new_tracked_faces[best_match_id] = {'rect': (x, y, w, h), 'last_seen_frame': frame_count}
            cv2.putText(rotated_frame, str(best_match_id), (x, y - 10),  # Correct position
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)  # Red color for ID
            cv2.rectangle(rotated_frame, (x, y, w, h), (0, 0, 255), 2)  # Red color for rectangle
        else:
            # Assign new ID
            new_tracked_faces[next_face_id] = {'rect': (x, y, w, h), 'last_seen_frame': frame_count}
            cv2.putText(rotated_frame, str(next_face_id), (x, y - 10),  # Correct position
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)  # Red color for ID
            cv2.rectangle(rotated_frame, (x, y, w, h), (0, 0, 255), 2)  # Red color for rectangle
            next_face_id += 1

    # Remove lost faces
    faces_to_remove = []
    for face_id, face_data in tracked_faces.items():
        if frame_count - face_data['last_seen_frame'] > ID_PERSISTENCE_FRAMES:
            faces_to_remove.append(face_id)
    for face_id in faces_to_remove:
        del tracked_faces[face_id]

    tracked_faces = new_tracked_faces # Update the tracked faces

     # Motor Control (Corrected)
    if tracked_faces:
        first_tracked_id = min(tracked_faces.keys())
        x, y, w, h = tracked_faces[first_tracked_id]['rect']

        # Calculate motor positions with offset (separate from drawing)
        motor_x = x + SHOULDER_OFFSET  # Offset applied ONLY for motor control
        motor_y = y + ELBOW_OFFSET # Offset applied ONLY for motor control

        shoulder_motor_position = int(np.interp(motor_x, [0, rotated_frame.shape[1]], [SHOULDER_MOTOR_MIN, SHOULDER_MOTOR_MAX]))
        elbow_motor_position = int(np.interp(motor_y, [0, rotated_frame.shape[0]], [ELBOW_MOTOR_MIN, ELBOW_MOTOR_MAX]))

        center_x = rotated_frame.shape[1] // 2
        tolerance = 50
        motor_controller.control_shoulder_motor(motor_x, center_x, tolerance)  # Use motor_x
        center_y = rotated_frame.shape[0] // 2
        tolerance_y = 50
        motor_controller.control_elbow_motor(motor_y, center_y, tolerance_y)  # Use motor_y

        # Draw the rectangle (RED) and display coordinates (using original x, y)
        cv2.rectangle(rotated_frame, (x, y, w, h), (0, 0, 255), 2)  # Red color
        coordinates_text = f"X: {x}, Y: {y}"
        cv2.putText(rotated_frame, coordinates_text, (x, y + h + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)  # Red color
                    
    end_time = time.time()
    fps = 1 / (end_time - start_time)

    frame_count += 1  # Increment frame counter

    if frame_count % FPS_DISPLAY_INTERVAL == 0:  # Check if it's time to display FPS
        print(f"Frame rate: {fps:.2f} fps")
        frame_count = 0  # Reset frame counter

    # Display frame rate
    #print(f"Frame rate: {fps:.2f} fps")
    display_frame = cv2.resize(rotated_frame, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE, interpolation=cv2.INTER_LINEAR)

    cv2.imshow("Face Detection (Rotated)", display_frame)  # Show the larger frame
    
     # Convert the frame to RGB (important for v4l2loopback)
    output_frame = cv2.cvtColor(rotated_frame, cv2.COLOR_BGR2RGB)

    # Write the frame to the pipe
    #pipe.stdin.write(output_frame.tobytes())

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows() 


