import cv2
import time
import numpy as np  # Import numpy
from motor_control import MotorController  # Import your motor control class

FPS_DISPLAY_INTERVAL = 10  # Display FPS every 10 frames
frame_count = 0  # Initialize frame counter
CAMERA_WIDTH = 320  # Reduced width for faster processing
CAMERA_HEIGHT = 240  # Reduced height for faster processing
# Open the default camera (video0)
cap = cv2.VideoCapture("/dev/video0")
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

# Load a pre-trained face detection model (Haar cascade - faster)
face_cascade = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml") # Adjust path if needed

# Motor position mapping ranges (adjust these to your motor's limits)
SCS_SHOULDER_HOME_POSITION_VALUE = 2000
SHOULDER_MOTOR_MIN = 1000  # Replace with your actual minimum value
SHOULDER_MOTOR_MAX = 2700  # Replace with your actual maximum value
SCS_ELBOW_HOME_POSITION_VALUE = 2000
ELBOW_MOTOR_MIN = 1678     # Replace with your actual minimum value
ELBOW_MOTOR_MAX = 2250   # Replace with your actual maximum value

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

# Variables for no-face detection timer
no_face_start_time = None
NO_FACE_TIMEOUT = 5  # seconds

no_face_start_time = None
NO_FACE_TIMEOUT =   # Seconds

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    scale_factor = 0.5
    resized_frame = cv2.resize(frame, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_LINEAR)

    rotated_frame = cv2.transpose(resized_frame)
    rotated_frame = cv2.flip(rotated_frame, flipCode=1)

    start_time = time.time()

    faces = face_cascade.detectMultiScale(
        rotated_frame,
        scaleFactor=1.07,
        minNeighbors=4,
        flags=cv2.CASCADE_SCALE_IMAGE,
        minSize=(20, 20)
    )

    end_time = time.time()
    fps = 1 / (end_time - start_time)

    first_face_coords = None
    if len(faces) > 0:  # Check if any faces were detected
        if no_face_start_time is not None:
            no_face_start_time = None  # Reset timer if face is found

        for i, (x, y, w, h) in enumerate(faces):
            cv2.rectangle(rotated_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            if i == 0:
                first_face_coords = (x, y, w, h)

            cv2.putText(rotated_frame, str(face_id), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            if i == 0:
                first_face_coords = (x, y, w, h)

            face_id += 1
        face_id = 0

        if first_face_coords:
            x, y, w, h = first_face_coords

            shoulder_motor_position = int(np.interp(x, [0, rotated_frame.shape[1]], [SHOULDER_MOTOR_MIN, SHOULDER_MOTOR_MAX]))
            elbow_motor_position = int(np.interp(y, [0, rotated_frame.shape[0]], [ELBOW_MOTOR_MIN, ELBOW_MOTOR_MAX]))

            center_x = rotated_frame.shape[1] // 2
            tolerance = 50
            motor_controller.control_shoulder_motor(x, center_x, tolerance)

            center_y = rotated_frame.shape[0] // 2
            tolerance_y = 50
            motor_controller.control_elbow_motor(y, center_y, tolerance_y)

    else:  # No faces detected
        if no_face_start_time is None:
            no_face_start_time = time.time()  # Start the timer

        elif time.time() - no_face_start_time >= NO_FACE_TIMEOUT:
            print("No faces detected for 30 seconds. Returning to home position.")
            motor_controller.set_shoulder_position(SHOULDER_HOME)  # Move to home
            motor_controller.set_elbow_position(ELBOW_HOME)      # Move to home
            no_face_start_time = None # Reset the timer after returning to home position.


    end_time = time.time()
    fps = 1 / (end_time - start_time)

    frame_count += 1

    if frame_count % FPS_DISPLAY_INTERVAL == 0:
        print(f"Frame rate: {fps:.2f} fps")
        frame_count = 0

    display_frame = cv2.resize(rotated_frame, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE, interpolation=cv2.INTER_LINEAR)

    cv2.imshow("Face Detection (Rotated)", display_frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
