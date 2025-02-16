import os
import cv2
import time
import numpy as np
from motor_control import MotorController
import subprocess

FPS_DISPLAY_INTERVAL = 10
frame_count = 0
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

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

face_cascade = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml")

tracked_faces = {}
next_face_id = 0
ID_PERSISTENCE_FRAMES = 100

SHOULDER_MOTOR_MIN = 1000
SHOULDER_MOTOR_MAX = 2700
ELBOW_MOTOR_MIN = 1678
ELBOW_MOTOR_MAX = 2250

SHOULDER_OFFSET = 50
ELBOW_OFFSET = 0

motor_controller = MotorController()

face_id = 0
DISPLAY_SCALE = 2
scale_factor = 0.5

SCS_SHOULDER_HOME_POSITION_VALUE = 2000  # From motor_control.py
SCS_ELBOW_HOME_POSITION_VALUE = 2000    # From motor_control.py

no_face_start_time = None
NO_FACE_TIMEOUT = 5  # Seconds

shoulder_at_home = False
elbow_at_home = False


while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

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

    new_tracked_faces = {}
    for x, y, w, h in faces:
        best_match_id = None
        min_distance = float('inf')

        for face_id, face_data in tracked_faces.items():
            last_x, last_y, _, _ = face_data['rect']
            distance = np.sqrt((x - last_x)**2 + (y - last_y)**2)
            if distance < min_distance:
                min_distance = distance
                best_match_id = face_id

        if best_match_id is not None and min_distance < 50:
            new_tracked_faces[best_match_id] = {'rect': (x, y, w, h), 'last_seen_frame': frame_count}
            cv2.putText(rotated_frame, str(best_match_id), (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)
            cv2.rectangle(rotated_frame, (x, y, w, h), (0, 0, 255), 2)
        else:
            new_tracked_faces[next_face_id] = {'rect': (x, y, w, h), 'last_seen_frame': frame_count}
            cv2.putText(rotated_frame, str(next_face_id), (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)
            cv2.rectangle(rotated_frame, (x, y, w, h), (0, 0, 255), 2)
            next_face_id += 1

    faces_to_remove = []
    for face_id, face_data in tracked_faces.items():
        if frame_count - face_data['last_seen_frame'] > ID_PERSISTENCE_FRAMES:
            faces_to_remove.append(face_id)
    for face_id in faces_to_remove:
        del tracked_faces[face_id]

    tracked_faces = new_tracked_faces

    if len(faces) > 0:  # Faces detected <--- START OF IF BLOCK
        if no_face_start_time is not None:  # Reset the timer when faces are detected
            no_face_start_time = None
            shoulder_at_home = False  # Reset flags
            elbow_at_home = False

        if tracked_faces:
            first_tracked_id = min(tracked_faces.keys())
            x, y, w, h = tracked_faces[first_tracked_id]['rect']

            motor_x = x + SHOULDER_OFFSET
            motor_y = y + ELBOW_OFFSET

            center_x = rotated_frame.shape[1] // 2
            tolerance = 50
            motor_controller.control_shoulder_motor(motor_x, center_x, tolerance)
            center_y = rotated_frame.shape[0] // 2
            tolerance_y = 50
            motor_controller.control_elbow_motor(motor_y, center_y, tolerance_y)

            cv2.rectangle(rotated_frame, (x, y, w, h), (0, 0, 255), 2)
            coordinates_text = f"X: {x}, Y: {y}"
            cv2.putText(rotated_frame, coordinates_text, (x, y + h + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)

    else:  # No faces detected
        if no_face_start_time is None:
            no_face_start_time = time.time()  # Start the timer

        elif time.time() - no_face_start_time >= NO_FACE_TIMEOUT:
            if not shoulder_at_home or not elbow_at_home: # Only move if not already at home
                print(f"No faces detected for {NO_FACE_TIMEOUT} seconds. Returning to home position.")

            motor_controller.go_home()
            shoulder_at_home = True
            elbow_at_home = True

    
            
            #no_face_start_time = None  # Don't reset here

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
