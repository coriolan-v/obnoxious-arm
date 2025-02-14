import cv2
import dlib
import numpy as np
import time

TRACKER_TIMEOUT = 0.5
FACE_CONFIDENCE_THRESHOLD = 0.8
FACE_MATCH_DISTANCE_THRESHOLD = 50

class FaceDetector:
    def __init__(self, model_file, config_file):
        self.net = cv2.dnn.readNetFromCaffe(model_file, config_file)
        self.trackers = []
        self.last_seen = {}
        self.unique_id = 0
        self.face_ids = {}
        self.frame_resized = None  # Store the resized frame
        self.mean = (104.0, 177.0, 123.0)  # Pre-calculate mean for blobFromImage
        self.input_size = (300, 300)      # Pre-define input size

    def detect_faces(self, frame):
        # Resize frame only once and store it
        self.frame_resized = cv2.resize(frame, self.input_size)
        blob = cv2.dnn.blobFromImage(self.frame_resized, 1.0, self.input_size, self.mean)
        self.net.setInput(blob)
        detections = self.net.forward()

        faces = []
        height, width = frame.shape[:2]  # Get original frame dimensions once
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > FACE_CONFIDENCE_THRESHOLD:
                box = detections[0, 0, i, 3:7] * np.array([width, height, width, height])  # Use pre-calculated dims
                x, y, w, h = box.astype("int")
                faces.append((x, y, int(w) - x, int(h) - y))  # Store as (x, y, width, height)
        return faces

    def update_trackers(self, frame):
        current_time = time.time()
        self.trackers = [
            tracker for tracker in self.trackers
            if (current_time - self.last_seen[self.face_ids[tracker]] <= TRACKER_TIMEOUT and tracker.update(frame) > 0)
        ]

    def match_and_track(self, frame, faces):
        tracked_positions = {
            self.face_ids[tracker]: (int(tracker.get_position().left()), int(tracker.get_position().top()),
                                      int(tracker.get_position().width()), int(tracker.get_position().height()))
            for tracker in self.trackers
        }

        for (x, y, w, h) in faces:
            best_match_id = None
            min_distance = FACE_MATCH_DISTANCE_THRESHOLD

            for tid, (tx, ty, tw, th) in tracked_positions.items():
                distance = calculate_distance((x, y, w, h), (tx, ty, tw, th))
                if distance < min_distance:
                    min_distance = distance
                    best_match_id = tid

            if best_match_id is None:  # No match found, create a new tracker
                tracker = dlib.correlation_tracker()
                tracker.start_track(frame, dlib.rectangle(x, y, x + w, y + h))
                self.trackers.append(tracker)
                self.face_ids[tracker] = self.unique_id
                self.last_seen[self.unique_id] = time.time()
                self.unique_id += 1
            else:  # Update the last seen time for matched tracker
                self.last_seen[best_match_id] = time.time()

    def get_tracked_positions(self):
        return {
            self.face_ids[tracker]: (int(tracker.get_position().left()), int(tracker.get_position().top()),
                                      int(tracker.get_position().width()), int(tracker.get_position().height()))
            for tracker in self.trackers
        }

    def draw_tracking_boxes(self, frame):
        for tracker in self.trackers:
            pos = tracker.get_position()
            x = int(pos.left())
            y = int(pos.top())
            w = int(pos.width())
            h = int(pos.height())
            tid = self.face_ids[tracker]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, f"ID: {tid}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    def get_first_face_coordinates(self):
        if self.trackers:
            tracker = self.trackers[0]
            pos = tracker.get_position()
            x = int(pos.left())
            y = int(pos.top())
            w = int(pos.width())
            h = int(pos.height())
            return x, y, w, h
        return None


def calculate_distance(rect1, rect2):
    center1 = np.array([rect1[0] + rect1[2] / 2, rect1[1] + rect1[3] / 2])
    center2 = np.array([rect2[0] + rect2[2] / 2, rect2[1] + rect2[3] / 2])
    return np.linalg.norm(center1 - center2)
