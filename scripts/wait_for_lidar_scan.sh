#!/bin/bash
# Script to wait for lidar scan topic before launching FAST-LIVO2
# This ensures the lidar is ready before starting the mapping node

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LIDAR_TOPIC="${1:-/rslidar_points}"
TIMEOUT="${2:-30}"

echo "========================================"
echo "Waiting for LiDAR to be ready..."
echo "========================================"

# Wait for the lidar topic
"$SCRIPT_DIR/wait_for_topic.sh" "$LIDAR_TOPIC" "$TIMEOUT"

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================"
    echo "✓ LiDAR is ready!"
    echo "========================================"
    exit 0
else
    echo ""
    echo "========================================"
    echo "✗ LiDAR failed to start in time"
    echo "========================================"
    exit 1
fi
