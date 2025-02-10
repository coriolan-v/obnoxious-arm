import cv2
import time

def display_webcam(device_index=0, timeout=5):
    """
    Displays the live video feed from the specified webcam with a timeout.

    Args:
        device_index (int): Index of the webcam to use (default: 2).
        timeout (float): Timeout in seconds for opening the webcam (default: 5).
    """
    camera_name = "/dev/video0"

    cap = cv2.VideoCapture(0)

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

        cv2.imshow('Webcam Feed', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    display_webcam()
