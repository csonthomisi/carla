#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Go to the root of the CARLA project (adjust if needed)
cd "$(dirname "$0")"

echo "Building PythonAPI..."
make PythonAPI

echo "Installing CARLA Python wheel..."
pip3 install PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-linux_x86_64.whl --force-reinstall

echo "✅ CARLA PythonAPI build and install complete."
