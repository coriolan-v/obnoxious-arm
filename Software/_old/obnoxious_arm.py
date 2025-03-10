import cv2
import os
from face_detection import FaceDetector
from motor_control import MotorController
import time

# Face detection models (replace with your paths)
MODEL_FILE = "deploy.prototxt"
CONFIG_FILE = "res10_300x300_ssd_iter_140000.caffemodel"

# Video capture settings
VIDEO_CAPTURE_INDEX = 0  # Or your camera index

# Frame cropping
TOP_HEIGHT = 250
BOTTOM_HEIGHT = 700

WIDTH = 480
HEIGHT = 640

# Window name
WINDOW_NAME = 'Video'

def main():
    # Initialize face detector and motor controller
    face_detector = FaceDetector(MODEL_FILE, CONFIG_FILE)
    motor_controller = MotorController()

    # Initialize video capture
    video_capture = cv2.VideoCapture(VIDEO_CAPTURE_INDEX, cv2.CAP_V4L2)
    if not video_capture.isOpened():
        print("Error: Could not open video capture device.")
        return

    #video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    #video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    
    
    # Main loop
    min_x = float('inf')
    max_x = float('-inf')
    min_y = float('inf')
    max_y = float('-inf')
    
    frame_count = 0
    
    last_face_detection_time = time.time()  # Keep track of the last detection time
    motor_sweep_enabled = False  # Flag to indicate if motor sweep is active
    sweep_direction = 1 # 1 for right, -1 for left
    sweep_speed = 50 # Adjust as needed
    
    while True:
        ret, frame = video_capture.read()
        if not ret: break

        # Rotate the frame 90 degrees clockwise
        rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)  # Or cv2.ROTATE_90_COUNTERCLOCKWISE
        #rotated_frame = frame

        # 1. Target Resolution (Experiment!)
        new_width = WIDTH
        new_height = HEIGHT

        # 2. Resize the *rotated* frame
        #resized_frame = cv2.resize(rotated_frame, (new_width, new_height))
        resized_frame = rotated_frame
        
        
        # 3. Face Detection and Tracking (using resized and rotated frame)
        faces = face_detector.detect_faces(resized_frame)
        face_detector.update_trackers(resized_frame)
        face_detector.match_and_track(resized_frame, faces)

        # Get and remap face coordinates
        first_face_coords = face_detector.get_first_face_coordinates()
        if first_face_coords:
            last_face_detection_time = time.time()  # Reset detection time
            motor_sweep_enabled = False  # Stop any sweep
            #motor_controller.stop_sweep()
            x, y, w, h = first_face_coords
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)

            if max_x - min_x != 0:
                remapped_x = (x - min_x) / (max_x - min_x)
            else:
                remapped_x = 0

            if max_y - min_y != 0:
                remapped_y = (y - min_y) / (max_y - min_y)
            else:
                remapped_y = 0

            print(f"Original y: {y}, Remapped y (0-1): {remapped_y}")
            print(f"Original x: {x}, Remapped x (0-1): {remapped_x}")

            # Motor control
            center_x = resized_frame.shape[1] // 2
            tolerance = 50
            motor_controller.control_shoulder_motor(x, center_x, tolerance)

            center_y = resized_frame.shape[0] // 2
            tolerance_y = 50
            motor_controller.control_elbow_motor(y, center_y, tolerance_y)
        
        #else:
            #if time.time() - last_face_detection_time > 3:  # 10 seconds without detection
                #motor_sweep_enabled = True  # Enable sweep
                #motor_controller.sweep_shoulder_motor()
                #print("No face detected for 10 seconds. Starting motor sweep.")
                

        

        # Draw tracking boxes
        face_detector.draw_tracking_boxes(resized_frame)
    
    
        cv2.imshow(WINDOW_NAME, resized_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Cleanup
    motor_controller.close_port()
    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
