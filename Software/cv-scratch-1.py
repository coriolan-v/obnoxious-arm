import cv2
import dlib
import numpy as np
import time

# Load the pre-trained model
model_file = "deploy.prototxt"
config_file = "res10_300x300_ssd_iter_140000.caffemodel"
net = cv2.dnn.readNetFromCaffe(model_file, config_file)

# Open a connection to the default camera (usually /dev/video0)
video_capture = cv2.VideoCapture(2)

# Set the width and height of the camera preview
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Initialize dlib's correlation tracker
trackers = []
last_seen = {}
timeout = 2  # seconds
unique_id = 0
face_ids = {}

def display_webcam(device_index=0, timeout=5, width=320, height=480):
    """
    Displays the live video feed from the specified webcam with a timeout and resized dimensions.

    Args:
        device_index (int): Index of the webcam to use (default: 0).
        timeout (float): Timeout in seconds for opening the webcam (default: 5).
        width (int): Desired width of the output frame.
        height (int): Desired height of the output frame.
    """

    cap = cv2.VideoCapture(device_index)

    # Add timeout for opening the webcam
    start_time = time.time()
    while not cap.isOpened() and time.time() - start_time < timeout:
        cap = cv2.VideoCapture(device_index)

    if not cap.isOpened():
        print(f"Error opening webcam {device_index} within {timeout} seconds.")
        return

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # Resize the frame
        resized_frame = cv2.resize(frame, (width, height))

        cv2.imshow('Webcam Feed', resized_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()



def calculate_distance(rect1, rect2):
    center1 = np.array([rect1[0] + rect1[2] / 2, rect1[1] + rect1[3] / 2])
    center2 = np.array([rect2[0] + rect2[2] / 2, rect2[1] + rect2[3] / 2])
    return np.linalg.norm(center1 - center2)

while True:
    # Capture frame-by-frame
    ret, frame = video_capture.read()

    # Convert the frame to a blob for the DNN model
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))

    # Set the blob as the input to the network
    net.setInput(blob)

    # Run the forward pass to get the face detections
    detections = net.forward()

    faces = []
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:  # Confidence threshold
            box = detections[0, 0, i, 3:7] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
            (x, y, w, h) = box.astype("int")
            faces.append((x, y, w - x, h - y))

    current_time = time.time()

    # Update existing trackers and remove timed-out trackers
    new_trackers = []
    for tracker in trackers:
        tracker.update(frame)
        tid = face_ids[tracker]
        if current_time - last_seen[tid] > timeout:
            del last_seen[tid]
            del face_ids[tracker]
        else:
            new_trackers.append(tracker)

    trackers = new_trackers

    # Check if the detected faces match existing trackers
    tracked_positions = [(tracker.get_position().left(), tracker.get_position().top(), tracker.get_position().width(),
                          tracker.get_position().height()) for tracker in trackers]

    for (x, y, w, h) in faces:
        match_found = False
        for (tx, ty, tw, th) in tracked_positions:
            if calculate_distance((x, y, w, h), (tx, ty, tw, th)) < 50:  # Threshold to consider it the same face
                match_found = True
                break
        if not match_found:
            tracker = dlib.correlation_tracker()
            tracker.start_track(frame, dlib.rectangle(x, y, x + w, y + h))
            trackers.append(tracker)
            face_ids[tracker] = unique_id
            last_seen[unique_id] = current_time
            unique_id += 1

    # Draw rectangles and unique IDs around tracked faces
    for tracker in trackers:
        pos = tracker.get_position()
        x = int(pos.left())
        y = int(pos.top())
        w = int(pos.width())
        h = int(pos.height())
        tid = face_ids[tracker]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)  # Red rectangle
        cv2.putText(frame, f"ID: {tid}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Display the resulting frame
    cv2.imshow('Video', frame)

    output_width = 320
    output_height = 480

    #display_webcam(width=output_width, height=output_height)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close the windows
video_capture.release()
cv2.destroyAllWindows()
