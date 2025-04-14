#!/bin/bash

echo "Installing dependencies for coffee_head face recognition..."

# Install required system packages
sudo apt-get update
sudo apt-get install -y \
    python3-opencv \
    python3-pip \
    python3-pyqt5 \
    python3-dlib

# Install OpenCV contrib with face module
pip3 install opencv-contrib-python

echo "Installation complete. Please rebuild the package with:"
echo "colcon build --packages-select coffee_head" 