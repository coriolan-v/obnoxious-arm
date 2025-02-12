import cv2
import dlib
import numpy as np
import time

TRACKER_TIMEOUT = 2
FACE_CONFIDENCE_THRESHOLD = 0.5
FACE_MATCH_DISTANCE_THRESHOLD = 50

class FaceDetector:
    def __init__(self, model_file, config_file):
        self.net = cv2.dnn.readNetFromCaffe(model_file, config_file)
        self.trackers = []
        self.last_seen = {}
        self.unique_id = 0
        self.face_ids = {}

    def detect_faces(self, frame):
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.net.setInput(blob)
        detections = self.net.forward()

        faces = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > FACE_CONFIDENCE_THRESHOLD:
                box = detections[0, 0, i, 3:7] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
                (x, y, w, h) = box.astype("int")
                faces.append((x, y, w - x, h - y))
        return faces

    def update_trackers(self, frame):
        current_time = time.time()
        new_trackers = []
        for tracker in self.trackers:
            tracker.update(frame)
            tid = self.face_ids[tracker]
            if current_time - self.last_seen[tid] > TRACKER_TIMEOUT:
                del self.last_seen[tid]
                del self.face_ids[tracker]
            else:
                new_trackers.append(tracker)
        self.trackers = new_trackers

    def match_and_track(self, frame, faces):
        tracked_positions = [(tracker.get_position().left(), tracker.get_position().top(), tracker.get_position().width(),
                             tracker.get_position().height()) for tracker in self.trackers]

        for (x, y, w, h) in faces:
            match_found = False
            for (tx, ty, tw, th) in tracked_positions:
                if calculate_distance((x, y, w, h), (tx, ty, tw, th)) < FACE_MATCH_DISTANCE_THRESHOLD:
                    match_found = True
                    break
            if not match_found:
                tracker = dlib.correlation_tracker()
                tracker.start_track(frame, dlib.rectangle(x, y, x + w, y + h))
                self.trackers.append(tracker)
                self.face_ids[tracker] = self.unique_id
                self.last_seen[self.unique_id] = time.time()
                self.unique_id += 1

    def get_tracked_positions(self):
        return [(tracker.get_position().left(), tracker.get_position().top(), tracker.get_position().width(),
                 tracker.get_position().height()) for tracker in self.trackers]

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
            first_tracker = self.trackers[0]
            pos = first_tracker.get_position()
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
