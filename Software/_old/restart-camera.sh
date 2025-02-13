#!/bin/bash

PACKAGE_NAME="com.sec.android.app.camera"
ACTIVITY_NAME=".Camera"  # Or whatever is correct

while true; do  # Loop indefinitely
    adb shell am start -n "$PACKAGE_NAME/$ACTIVITY_NAME"
    ACTIVITY_STATE=$(adb shell dumpsys activity activities | grep "$PACKAGE_NAME/$ACTIVITY_NAME")
    if [[ -z "$ACTIVITY_STATE" ]]; then
        echo "Camera activity not found. Launching..."
        adb shell am start -n "$PACKAGE_NAME/$ACTIVITY_NAME"
    else
        echo "Camera activity found. Hopefully, it's now visible."
    fi
    sleep 1  # Check every 5 seconds (adjust as needed)
done
