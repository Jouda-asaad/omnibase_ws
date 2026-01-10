#!/bin/bash
# Launch script for omnibase simulation with camera views
# - Fixes NVIDIA/Mesa EGL conflict by forcing NVIDIA EGL vendor
# - Launches simulation and camera debug views

set -e

# Force NVIDIA EGL to prevent Mesa segfault
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

# Source workspace
cd ~/omnibase_ws
source install/setup.bash

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down all processes..."
    pkill -P $$ 2>/dev/null || true
    wait 2>/dev/null
    echo "Done."
}
trap cleanup EXIT INT TERM

echo "=========================================="
echo "  Omnibase Simulation Launcher"
echo "=========================================="

# Launch simulation in background
echo "[1/4] Starting Gazebo simulation..."
ros2 launch omnibase_bringup full_simulation.launch.py "$@" &
SIM_PID=$!

# Wait for simulation and controllers to initialize
echo "[2/4] Waiting for controllers to initialize (10s)..."
sleep 10

# Check if mecanum controller is active
echo "[3/4] Checking controller status..."
if ros2 control list_controllers 2>/dev/null | grep -q "mecanum_controller.*active"; then
    echo "✓ Mecanum controller is active"
else
    echo "⚠ Warning: mecanum_controller may not be active yet"
fi

# Launch camera debug views
echo "[4/4] Starting camera views..."
ros2 run rqt_image_view rqt_image_view /camera/image_raw &
RAW_CAM_PID=$!

ros2 run rqt_image_view rqt_image_view /camera/debug &
DEBUG_CAM_PID=$!

echo ""
echo "=========================================="
echo "  Simulation Running"
echo "=========================================="
echo ""
echo "Camera views:"
echo "  - Raw camera: /camera/image_raw"
echo "  - Debug view: /camera/debug (with LED detection overlay)"
echo ""
echo "Test commands (run in new terminal):"
echo "  source ~/omnibase_ws/install/setup.bash"
echo ""
echo "  # Test forward motion:"
echo "  ros2 topic pub --once /mecanum_controller/cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.3}}\""
echo ""
echo "  # Test strafing:"
echo "  ros2 topic pub --once /mecanum_controller/cmd_vel geometry_msgs/msg/Twist \"{linear: {y: 0.3}}\""
echo ""
echo "Press Ctrl+C to stop all processes."
echo ""

# Wait for simulation to finish
wait $SIM_PID
