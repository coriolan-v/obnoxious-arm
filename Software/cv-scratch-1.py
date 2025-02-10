import cv2
import dlib
import numpy as np
import time

# Load the pre-trained model
model_file = "deploy.prototxt"
config_file = "res10_300x300_ssd_iter_140000.caffemodel"
net = cv2.dnn.readNetFromCaffe(model_file, config_file)

# Open a connection to the default camera (usually /dev/video0)
video_capture = cv2.VideoCapture(0, cv2.CAP_V4L2)

# Check if the video capture device is opened correctly
if not video_capture.isOpened():
    print("Error: Could not open video capture device.")
    exit()

# Set the width and height of the camera preview
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

# Initialize dlib's correlation tracker
trackers = []
last_seen = {}
timeout = 2  # seconds
unique_id = 0
face_ids = {}

def calculate_distance(rect1, rect2):
    center1 = np.array([rect1[0] + rect1[2] / 2, rect1[1] + rect1[3] / 2])
    center2 = np.array([rect2[0] + rect2[2] / 2, rect2[1] + rect2[3] / 2])
    return np.linalg.norm(center1 - center2)

# Create a named window
cv2.namedWindow('Video', cv2.WINDOW_NORMAL)

while True:
    ret, frame = video_capture.read()

    if not ret:
        print("Error: Could not read frame.")
        break

    # Get the screen size after the window has been created
    screen_width, screen_height = cv2.getWindowImageRect('Video')[2:4]

    # Calculate the aspect ratio of the original frame
    frame_height, frame_width = frame.shape[:2]
    aspect_ratio = frame_width / frame_height

    # Adjust the dimensions to maintain the aspect ratio
    if screen_width / screen_height > aspect_ratio:
        new_width = int(screen_height * aspect_ratio)
        new_height = screen_height
    else:
        new_width = screen_width
        new_height = int(screen_width / aspect_ratio)

    # Resize the frame to fit the screen size while maintaining the aspect ratio
    resized_frame = cv2.resize(frame, (new_width, new_height))

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

    for tracker in trackers:
        pos = tracker.get_position()
        x = int(pos.left())
        y = int(pos.top())
        w = int(pos.width())
        h = int(pos.height())
        tid = face_ids[tracker]
        cv2.rectangle(resized_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(resized_frame, f"ID: {tid}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow('Video', resized_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()

