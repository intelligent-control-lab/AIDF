#!/usr/bin/env bash
set -e

AIDF_ROOT="/root/catkin_ws/src/AIDF"
BACKEND_LOG="/tmp/aidf_web_backend.log"
FRONTEND_LOG="/tmp/aidf_web_frontend.log"

if [[ ! -d "$AIDF_ROOT" ]]; then
    echo "AIDF source not found at $AIDF_ROOT"
    exit 1
fi

# Load ROS and workspace environment for rosrun/roslaunch calls from the Flask backend.
source /opt/ros/noetic/setup.bash

cd /root/catkin_ws
catkin build aidf --no-status

if [[ -f /root/catkin_ws/devel/setup.bash ]]; then
    source /root/catkin_ws/devel/setup.bash
fi

cd "$AIDF_ROOT"
python3 webpage/server.py >"$BACKEND_LOG" 2>&1 &
BACKEND_PID=$!

python3 -m http.server --bind 0.0.0.0 8000 >"$FRONTEND_LOG" 2>&1 &
FRONTEND_PID=$!

echo "AIDF webpage backend started on http://localhost:5000"
echo "AIDF webpage frontend started on http://localhost:8000/webpage/api.html"
echo "Backend log: $BACKEND_LOG"
echo "Frontend log: $FRONTEND_LOG"

aidf_cleanup() {
    kill "$BACKEND_PID" "$FRONTEND_PID" 2>/dev/null || true
}

trap aidf_cleanup EXIT INT TERM
wait -n "$BACKEND_PID" "$FRONTEND_PID"
