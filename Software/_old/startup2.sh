#!/bin/bash

# Load v4l2loopback module
sudo modprobe v4l2loopback

#./scrcpy --video-source=camera --camera-id=0 --orientation=90 --camera-ar=4:3 

SCRCPY_PATH="//home/live/Documents/GitHub/obnoxious-arm/Software/scrcpy-linux-x86_64-v3.1/./scrcpy"
V4L2_DEVICE="/dev/video0"
CAMERA_SIZE="640x480"
ORIENTATION="90"
SPEED="120"
AR="4:3"

#2400x1080
#1920x1080
#        - 1440x1080
 #       - 1280x960
  #      - 1280x720
  #      - 1088x1088
  #      - 960x720
  #      - 720x480
  #      - 640x480
        #- 512x384
        #- 512x288
        #- 384x384
        #352x288
#320x240
#256x144
#176x144

run_scrcpy() {
  $SCRCPY_PATH --video-source=camera --no-audio --camera-size="$CAMERA_SIZE" --orientation="$ORIENTATION" --camera-fps="$SPEED" --v4l2-sink="$V4L2_DEVICE" 
  return $?
}

#  "$SCRCPY_PATH" --video-source=camera --no-audio --camera-size="$CAMERA_SIZE" --orientation="$ORIENTATION" --camera-fps="$SPEED" --camera-ar="$AR" --v4l2-sink="$V4L2_DEVICE" 
#  "$SCRCPY_PATH" --video-source=camera --no-audio --orientation="$ORIENTATION" --camera-fps="$SPEED" --camera-ar="$AR" --v4l2-sink="$V4L2_DEVICE" 

while true; do
  echo "Starting scrcpy..."
  run_scrcpy

  exit_code=$?

  if [[ $exit_code -eq 0 ]]; then
    echo "scrcpy started successfully."
    break  # Exit the loop if scrcpy starts.
  elif [[ $exit_code -eq 1 ]]; then # Check if ADB device is found
      echo "Scrcpy failed to start. Check ADB connection. Retrying in 5 seconds..."
      sleep 1
  else
    echo "scrcpy exited with an error (exit code: $exit_code). Retrying in 5 seconds..."
    sleep 1
  fi
done

echo "Script finished."

