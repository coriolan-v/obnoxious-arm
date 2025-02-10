import cv2

def detect_faces(frame):
    """
    Detects faces in the given frame using a more robust approach.

    Args:
        frame: The input image frame.

    Returns:
        A list of detected faces, where each face is represented as a tuple 
        containing the (x, y) coordinates of the top-left corner, 
        the width (w), and the height (h) of the bounding box.
    """

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Adjust parameters for better detection
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=4, minSize=(30, 30)) 

    return faces

def display_detections(frame, faces):
    """
    Draws rectangles around the detected faces and displays information.

    Args:
        frame: The input image frame.
        faces: A list of detected faces.
    """

    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)  # Red rectangle
        cv2.putText(frame, f"Face Detected: ({x}, {y})", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

# Open the default camera
cap = cv2.VideoCapture(0)

# Set camera resolution to a suitable value (e.g., 1280x720)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) 
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read frame from camera.")
        break

    # Calculate desired portrait dimensions (e.g., 960x1280)
    height, width, _ = frame.shape
    if width > height: 
        # Landscape orientation, crop to portrait
        desired_height = int(width * 1.5)  # Adjust the multiplier (1.5) for desired height
        if desired_height > height:
            desired_height = height
        crop_width = desired_height
        crop_x = (width - desired_height) // 2 
        frame = frame[:, crop_x:crop_x+crop_width, :] 

    # Detect faces
    faces = detect_faces(frame)

    # Display detections
    display_detections(frame, faces)

    # Display the resulting frame
    cv2.imshow('Face Detection (Phone Preview)', frame)

    # Exit if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()
