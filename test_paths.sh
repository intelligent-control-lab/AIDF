#!/bin/bash

# Test script to verify path resolution works correctly

echo "=== AIDF Path Resolution Test ==="
echo

# Test 1: Check if we can find the project root
echo "Test 1: Finding project root..."
cd /home/philip/catkin_ws/src/AIDF

# Test with different working directories
echo "  From project root:"
pwd
if [ -f "config/lego_tasks/skillgraph.json" ]; then
    echo "  ✓ skillgraph.json found"
else
    echo "  ✗ skillgraph.json NOT found"
fi

echo "  From subdirectory:"
cd exe
pwd
if [ -f "../config/lego_tasks/skillgraph.json" ]; then
    echo "  ✓ skillgraph.json found (relative)"
else
    echo "  ✗ skillgraph.json NOT found"
fi

cd ..

echo

# Test 2: Environment variable
echo "Test 2: Environment variable..."
export AIDF_ROOT_DIR="/home/philip/catkin_ws/src/AIDF"
echo "  AIDF_ROOT_DIR = $AIDF_ROOT_DIR"
if [ -d "$AIDF_ROOT_DIR/config" ]; then
    echo "  ✓ Config directory accessible"
else
    echo "  ✗ Config directory NOT accessible"
fi

echo

# Test 3: Setup script
echo "Test 3: Setup script..."
if [ -f "setup_env.sh" ]; then
    echo "  ✓ setup_env.sh exists"
    echo "  Running setup script..."
    source ./setup_env.sh
else
    echo "  ✗ setup_env.sh NOT found"
fi

echo

echo "=== Path Resolution Test Complete ==="
