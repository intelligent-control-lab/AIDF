# AIDF Path Configuration

This document explains how the AIDF project handles file paths to make it portable across different systems.

## Overview

The AIDF project now uses dynamic path resolution to eliminate hardcoded paths. This makes the project work seamlessly across different computers and directory structures.

## How It Works

### Path Resolution Priority

The `PathResolver` utility finds the project root using this priority order:

1. **Environment Variable**: `AIDF_ROOT_DIR` if set
2. **Executable Location**: Searches up from the executable path for project markers
3. **Current Directory**: Searches up from current working directory
4. **Common Patterns**: Tries common catkin workspace locations
5. **Fallback**: Uses current directory

### Project Markers

The system identifies the AIDF project root by looking for:
- `config/` directory
- `CMakeLists.txt` file
- `package.xml` file
- `config/lego_tasks/` subdirectory

## Setup Methods

### Method 1: Environment Variable (Recommended)

Set the `AIDF_ROOT_DIR` environment variable:

```bash
# Quick setup for current session
export AIDF_ROOT_DIR="/path/to/your/catkin_ws/src/AIDF"

# Or use the provided setup script
source /path/to/AIDF/setup_env.sh

# Make permanent by adding to ~/.bashrc
echo 'export AIDF_ROOT_DIR="/path/to/your/catkin_ws/src/AIDF"' >> ~/.bashrc
```

### Method 2: Run from Project Directory

Simply run executables from within the AIDF project tree. The system will automatically find the project root.

### Method 3: Use Command Line Arguments

Override paths with command line arguments:

```bash
# For plan_lego
./plan_lego /custom/path/to/skillgraph.json

# For webplan_lego
./webplan_lego /custom/path/to/skillgraph.json /custom/path/to/web_message.json
```

## Configuration Files

### skillgraph.json

The `rootPwd` field now supports dynamic placeholders:

```json
{
    "environment": {
        "rootPwd": "${AIDF_PROJECT_ROOT}/",
        ...
    }
}
```

This automatically resolves to the detected project root.

## File Structure

```
AIDF/
├── config/
│   ├── lego_tasks/
│   │   ├── skillgraph.json          # Main configuration
│   │   ├── robot_properties/        # Robot calibration files
│   │   └── assembly_tasks/          # Task definitions
│   └── web_message.json             # Web interface config
├── exe/
│   ├── plan_lego.cpp               # Uses PathResolver
│   └── webplan_lego.cpp            # Uses PathResolver
├── skillgraph/
│   ├── include/Utils/PathUtils.hpp  # Path resolution utilities
│   └── src/Utils/PathUtils.cpp
└── setup_env.sh                    # Environment setup script
```

## Troubleshooting

### Path Not Found Errors

1. **Check Environment Variable**:
   ```bash
   echo $AIDF_ROOT_DIR
   ```

2. **Verify Project Structure**:
   Ensure your AIDF directory contains the `config/` folder with the expected files.

3. **Check Permissions**:
   Ensure you have read access to the configuration files.

4. **Enable Debug Logging**:
   The PathResolver logs its search process, check the output for clues.

### Common Issues

- **Multiple AIDF copies**: If you have multiple copies, ensure `AIDF_ROOT_DIR` points to the correct one
- **Symlinks**: The system follows symlinks, but ensure the target structure is correct
- **Case sensitivity**: File paths are case-sensitive on Linux

## Migration from Old Hardcoded Paths

If you have old configuration files with hardcoded paths:

1. Update `skillgraph.json` to use `"rootPwd": "${AIDF_PROJECT_ROOT}/"`
2. Remove any hardcoded paths from your launch scripts
3. Use the new executables with automatic path resolution

## Example Usage

```bash
# Set up environment
source ./setup_env.sh

# Run with automatic path detection
./build/plan_lego

# Or specify custom paths
./build/plan_lego ./config/custom_skillgraph.json

# Web-enabled version
./build/webplan_lego
```
