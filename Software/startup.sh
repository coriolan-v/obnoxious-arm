#!/bin/bash

# Path to your scrcpy executable
SCRCPY_PATH="/home/live/_DEV/obnoxious-arm/Software/scrcpy-linux-x86_64-v3.1/scrcpy"

# V4l2 device
V4L2_DEVICE="/dev/video0"

# Function to run scrcpy and check the return code
run_scrcpy() {
  "$SCRCPY_PATH" --v4l2-sink="$V4L2_DEVICE"--no-video-playback 
  return $? # Return the exit code of scrcpy
}

# Loop indefinitely
while true; do
  echo "Starting scrcpy..."
  run_scrcpy

  exit_code=$?

  if [[ $exit_code -eq 0 ]]; then
    echo "scrcpy exited normally."
    break  # Exit the loop if scrcpy exits cleanly (you might want to remove this if you expect it to exit and restart regularly).
  elif [[ $exit_code -eq 143 ]]; then # Signal 15 (SIGTERM) or 2 (SIGINT)
      echo "Scrcpy terminated by user (Ctrl+C). Exiting."
      exit 0
  else
    echo "scrcpy crashed or exited with an error (exit code: $exit_code). Restarting" #in 5 seconds..."
    #sleep 5
  fi
done

echo "Script finished."
