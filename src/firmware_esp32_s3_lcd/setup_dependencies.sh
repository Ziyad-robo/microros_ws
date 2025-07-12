#!/bin/bash

# Setup script for micro-ROS PlatformIO dependencies
# This script installs all required Python dependencies in the PlatformIO environment

echo "Setting up micro-ROS PlatformIO dependencies..."

# Check if PlatformIO is installed
if ! command -v pio &> /dev/null; then
    echo "Error: PlatformIO is not installed. Please install it first."
    exit 1
fi

# Get PlatformIO Python environment path
PLATFORMIO_PYTHON="/home/$(whoami)/.platformio/penv/bin/python"

# Check if PlatformIO Python environment exists
if [ ! -f "$PLATFORMIO_PYTHON" ]; then
    echo "Error: PlatformIO Python environment not found at $PLATFORMIO_PYTHON"
    echo "Please make sure PlatformIO is properly installed."
    exit 1
fi

echo "Installing dependencies in PlatformIO environment..."

# Install dependencies from requirements.txt
$PLATFORMIO_PYTHON -m pip install -r requirements.txt

if [ $? -eq 0 ]; then
    echo "✅ All dependencies installed successfully!"
    echo "You can now run 'pio run' to build the micro-ROS firmware."
else
    echo "❌ Failed to install dependencies. Please check the error messages above."
    exit 1
fi

echo ""
echo "Dependency installation complete!"
echo "Next steps:"
echo "1. Run 'pio run' to build the firmware"
echo "2. Run 'pio run --target upload' to upload to ESP32" 