import cv2

import os
model_path = "face_detection_yunet_2022mar.onnx" # Or your path
if not os.path.exists(model_path):
    print(f"Error: Model file not found at {model_path}")
    exit()  # Or handle the error appropriately

face_detector = cv2.FaceDetectorYN.create(model=model_path, config="", input_size=(320, 320))
# Open the video capture from /dev/video0
cap = cv2.VideoCapture("/dev/video0")

if not cap.isOpened():
    print("Error: Could not open video capture device.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("End of video stream or error.")
        break

    height, width, _ = frame.shape

    face_detector.setInputSize((width, height))

    faces = face_detector.detect(frame)[1]

    if faces is not None:
        for box in faces:
            x1, y1, x2, y2, confidence = box[:5]
            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)

            if confidence > 0.5:  # Adjust confidence threshold as needed
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                landmarks = box[5:].reshape(-1, 2)
                for lx, ly in landmarks:
                    lx = int(lx)
                    ly = int(ly)
                    cv2.circle(frame, (lx, ly), 2, (0, 0, 255), -1)

    cv2.imshow("Face Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
        break

cap.release()
cv2.destroyAllWindows()
