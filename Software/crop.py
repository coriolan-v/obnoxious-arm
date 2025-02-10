import cv2

# Open a connection to the default camera (usually /dev/video0)
video_capture = cv2.VideoCapture(0, cv2.CAP_V4L2)

# Check if the video capture device is opened correctly
if not video_capture.isOpened():
    print("Error: Could not open video capture device.")
    exit()

cv2.namedWindow('Video Feed')

def update_crop(x):
    pass

# Create trackbars to adjust cropping parameters
cv2.createTrackbar('X Start', 'Video Feed', 0, 640, update_crop)
cv2.createTrackbar('Y Start', 'Video Feed', 0, 480, update_crop)
cv2.createTrackbar('Width', 'Video Feed', 100, 640, update_crop)
cv2.createTrackbar('Height', 'Video Feed', 100, 480, update_crop)

while True:
    ret, frame = video_capture.read()

    if not ret:
        print("Error: Could not read frame.")
        break

    # Get cropping parameters from trackbars
    x_start = cv2.getTrackbarPos('X Start', 'Video Feed')
    y_start = cv2.getTrackbarPos('Y Start', 'Video Feed')
    width = cv2.getTrackbarPos('Width', 'Video Feed')
    height = cv2.getTrackbarPos('Height', 'Video Feed')

    # Ensure cropping parameters are within the frame dimensions
    x_end = min(x_start + width, frame.shape[1])
    y_end = min(y_start + height, frame.shape[0])
    cropped_frame = frame[y_start:y_end, x_start:x_end]

    cv2.imshow('Video Feed', cropped_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()

