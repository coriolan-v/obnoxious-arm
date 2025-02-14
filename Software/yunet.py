# Import the necessary libraries
import depthai as dai
import cv2

# Create a pipeline
pipeline = dai.Pipeline()

# Create a color camera
camera = pipeline.create(dai.Camera())
camera.setPreviewSize(640, 480)
camera.setImageFormat(dai.ImageFormat.COLOR_BGR)
camera.setFps(30)
outputs = camera.out

# Create a Yolo model
model = pipeline.create(dai.Yolo(model_path="yunet.xml", input_size=640, num_classes=20, confidence_threshold=0.5))
model.input.connect(camera.out)
model.out.connect(pipeline.out)

# Create a display
display = dai.Device.createDisplay()

# Start the pipeline
pipeline.start()

# Loop until the user presses 'q'
while True:
    # Get the frame from the pipeline
    frame = pipeline.get(dai.ImgFrame()).getCvImage()

    # Get the detections from the model
    detections = model.get(dai.YoloDetections()).detections

    # Draw the detections on the frame
    for detection in detections:
        cv2.rectangle(frame, (detection.xmin, detection.ymin), (detection.xmax, detection.ymax), (0, 255, 0), 2)
        cv2.putText(frame, str(detection.label), (detection.xmin + 5, detection.ymin + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))

    # Show the frame
    display.show(frame)

    # Wait for a key press
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# Stop the pipeline
pipeline.stop()
