import cv2
import time
import numpy as np  # Import numpy
from motor_control import MotorController  # Import your motor control class

# Open the default camera (video0)
cap = cv2.VideoCapture("/dev/video0")
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()


# Load a pre-trained face detection model (Haar cascade - faster)
face_cascade = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml") # Adjust path if needed

# Motor position mapping ranges (adjust these to your motor's limits)
SHOULDER_MOTOR_MIN = 1000  # Replace with your actual minimum value
SHOULDER_MOTOR_MAX = 2700  # Replace with your actual maximum value
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

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break


    # Rotate the frame (same as before
    rows, cols, _ = frame.shape
    rotation_matrix = cv2.getRotationMatrix2D((cols / 2, rows / 2), -90, 1)
    rotated_frame = cv2.warpAffine(frame, rotation_matrix, (rows, cols))

    # Resize the rotated frame (same as before)
    scale_factor = 0.5
    resized_frame = cv2.resize(rotated_frame, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_LINEAR)
    start_time = time.time()

    # Face detection (same as before)
    faces = face_cascade.detectMultiScale(resized_frame, 1.1, 3, cv2.CASCADE_SCALE_IMAGE, minSize=(30, 30))
    end_time = time.time()
    fps = 1 / (end_time - start_time)

    # Draw bounding boxes and add IDs
    first_face_coords = None  # Initialize to None
    for i, (x, y, w, h) in enumerate(faces):  # Enumerate to get index
        x = int(x / scale_factor)
        y = int(y / scale_factor)
        w = int(w / scale_factor)
        h = int(h / scale_factor)
        cv2.rectangle(rotated_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Add face ID next to the bounding box
        cv2.putText(rotated_frame, str(face_id), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        if i == 0:  # Check if it's the first face (index 0)
            first_face_coords = (x, y, w, h)  # Store coordinates of the first face

        face_id += 1
    face_id = 0  # Reset for the next frame

    # Print or use the coordinates of the first face
    if first_face_coords:
        #print(f"Coordinates of the first face: {first_face_coords}")

        # Motor Control Logic using MotorController class
        x, y, w, h = first_face_coords

        # Map x and y to motor positions
        shoulder_motor_position = int(np.interp(x, [0, rotated_frame.shape[1]], [SHOULDER_MOTOR_MIN, SHOULDER_MOTOR_MAX]))
        elbow_motor_position = int(np.interp(y, [0, rotated_frame.shape[0]], [ELBOW_MOTOR_MIN, ELBOW_MOTOR_MAX]))

        #print(f"Shoulder Motor Position: {shoulder_motor_position}")
        #print(f"Elbow Motor Position: {elbow_motor_position}")

        center_x = rotated_frame.shape[1] // 2  # Center of the ROTATED frame
        tolerance = 50
        motor_controller.control_shoulder_motor(x, center_x, tolerance)  # Use MotorController

        center_y = rotated_frame.shape[0] // 2  # Center of the ROTATED frame
        tolerance_y = 50
        motor_controller.control_elbow_motor(y, center_y, tolerance_y)  # Use MotorController

    # Display frame rate
    print(f"Frame rate: {fps:.2f} fps")
    cv2.imshow("Face Detection (Rotated)", rotated_frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows() 
