#!/bin/bash

# Full command to run
STREAM_ROS="/home/stephanie/mfi_ddb_library_bug17/.venv/bin/python3 /home/stephanie/mfi_ddb_library_bug17/examples/stream_ros.py"

# Start tmux session running the command
tmux new-session -d -s STREAM_DDB "$STREAM_ROS"
