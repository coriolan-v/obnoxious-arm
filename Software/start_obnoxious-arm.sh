#!/bin/bash
sudo chmod 666 /dev/ttyUSB0
gnome-terminal -- bash -c "python3 /home/live/Documents/GitHub/obnoxious-arm/Software/obnoxious-arm.py; exec bash"
