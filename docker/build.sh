#!/bin/bash

# Script to build Docker image with GitHub credentials

# Exit on any error
set -e

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to check if a file exists
check_file() {
    if [ ! -f "$1" ]; then
        echo -e "${RED}Error: File $1 not found${NC}"
        exit 1
    fi
}

# Check for GitHub token
if [ -z "$GITHUB_TOKEN" ]; then
    echo -e "${RED}Error: GITHUB_TOKEN environment variable not set${NC}"
    exit 1
fi

# Build the Docker image
echo -e "${GREEN}Building Docker image with credentials...${NC}"
docker build \
    --build-arg GITHUB_TOKEN="$GITHUB_TOKEN" \
    -t aidf-image .

echo -e "${GREEN}Docker image built successfully!${NC}"