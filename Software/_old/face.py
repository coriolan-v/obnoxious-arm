import cv2
from ultralytics import YOLO
import time

# Load the YOLOv8 model
model = YOLO('yolov8n.pt')  # Or the path to your custom model

# Video capture (replace with your video source)
cap = cv2.VideoCapture('/dev/video0')  # Or a video file path
if not cap.isOpened():
    print("Error opening video capture")
    exit()

frame_rate_check_interval = 5  # Check frame rate every N frames
frame_count = 0
prev_frame_time = 0
new_frame_time = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1

    # YOLOv8 inference
    results = model(frame)  # Run object detection on the frame

    # Process the results
    for result in results:
        boxes = result.boxes  # Boxes object for detected objects
        for box in boxes:
            if box.cls == 0:  # Assuming class 0 is face (check your model training if different)
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get box coordinates
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw bounding box

    # Calculate and display frame rate every frame_rate_check_interval frames
    if frame_count % frame_rate_check_interval == 0:
        new_frame_time = time.time()
        fps = 1/(new_frame_time-prev_frame_time)
        prev_frame_time = new_frame_time
        fps = int(fps)
        fps = str(fps)
        cv2.putText(frame, fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 3, (100, 255, 0), 3, cv2.LINE_AA)
        frame_count = 0 # Reset frame count after checking

    # Display the frame
    cv2.imshow('YOLOv8 Face Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
