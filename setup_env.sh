#!/bin/bash

# AIDF Environment Setup Script
# This script sets up the necessary environment variables for the AIDF project

# Find the script directory (where this script is located)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Set AIDF_ROOT_DIR to the project root (script should be in project root)
export AIDF_ROOT_DIR="$SCRIPT_DIR"

echo "AIDF Environment Setup Complete!"
echo "AIDF_ROOT_DIR set to: $AIDF_ROOT_DIR"

# Verify the setup
if [ -d "$AIDF_ROOT_DIR/config" ]; then
    echo "✓ Config directory found"
else
    echo "⚠ Warning: Config directory not found. Please check your setup."
fi

if [ -f "$AIDF_ROOT_DIR/config/lego_tasks/skillgraph.json" ]; then
    echo "✓ Skillgraph config found"
else
    echo "⚠ Warning: Skillgraph config not found. Please check your setup."
fi

# Add this directory to PATH so executables can be found
export PATH="$AIDF_ROOT_DIR/build:$PATH"

echo ""
echo "To use this setup in your current shell, run:"
echo "source $(basename $0)"
echo ""
echo "To make this permanent, add the following to your ~/.bashrc:"
echo "export AIDF_ROOT_DIR=\"$AIDF_ROOT_DIR\""
echo "export PATH=\"\$AIDF_ROOT_DIR/build:\$PATH\""
