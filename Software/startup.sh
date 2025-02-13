#!/bin/bash

# Load v4l2loopback module
sudo modprobe v4l2loopback

SCRCPY_PATH="//home/live/Documents/GitHub/obnoxious-arm/Software/scrcpy-linux-x86_64-v3.1/./scrcpy"
V4L2_DEVICE="/dev/video0"
CAMERA_SIZE="640x480"
ORIENTATION="90"
SPEED="120"

run_scrcpy() {
  "$SCRCPY_PATH" --video-source=camera --no-audio --camera-size="$CAMERA_SIZE" --orientation="$ORIENTATION" --camera-fps="$SPEED" --v4l2-sink="$V4L2_DEVICE"
  return $?
}

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
