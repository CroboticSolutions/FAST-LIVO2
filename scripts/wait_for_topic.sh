#!/bin/bash
# Script to wait for a ROS topic to be published
# Usage: ./wait_for_topic.sh <topic_name> [timeout_seconds]

TOPIC_NAME="$1"
TIMEOUT="${2:-30}"  # Default timeout: 30 seconds

if [ -z "$TOPIC_NAME" ]; then
    echo "Usage: $0 <topic_name> [timeout_seconds]"
    echo "Example: $0 /rslidar_points 30"
    exit 1
fi

echo "Waiting for topic: $TOPIC_NAME (timeout: ${TIMEOUT}s)"

ELAPSED=0
INTERVAL=0.5

while [ $ELAPSED -lt $TIMEOUT ]; do
    # Check if topic exists
    if rostopic list | grep -q "^${TOPIC_NAME}$"; then
        echo "✓ Topic $TOPIC_NAME is now available!"

        # Wait for at least one message
        echo "Waiting for first message on $TOPIC_NAME..."
        timeout 5 rostopic echo "$TOPIC_NAME" -n 1 > /dev/null 2>&1

        if [ $? -eq 0 ]; then
            echo "✓ Received message on $TOPIC_NAME"
            exit 0
        else
            echo "⚠ Topic exists but no messages received yet, continuing to wait..."
        fi
    fi

    sleep $INTERVAL
    ELAPSED=$(echo "$ELAPSED + $INTERVAL" | bc)

    # Print progress every 5 seconds
    if [ $(echo "$ELAPSED % 5" | bc) -eq 0 ]; then
        echo "Still waiting... (${ELAPSED}s elapsed)"
    fi
done

echo "✗ Timeout waiting for topic $TOPIC_NAME after ${TIMEOUT}s"
exit 1
