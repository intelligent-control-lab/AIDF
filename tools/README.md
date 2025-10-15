### AIDF Tools Overview

This directory contains optional tools and research submodules used alongside the core AIDF skill-graph planner. Each tool is maintained in its own repository and can be used independently. Below is a brief description of each tool, how/where it fits within this repo, and where to get help.

---

### Dinov2-Nist-Detection
- **What it does**: DINOv2-based visual matching to detect NIST box components, estimate 3D pose, and publish robot control topics via ROS. Provides edge/center detection and basic clustering/pose estimation utilities.
- **Key entry points**: `run_dino.py`, `scripts/` (ROS helpers, robot interface), `robot_control.py`.
- **Where it’s used in AIDF**: Used as a standalone perception/robot-control helper. It publishes ROS topics (e.g., detection and orientation) that can be consumed by downstream nodes during real-robot experiments. It is not tightly coupled to the C++ planner code but is intended to complement it in practical setups.
- **Upstream**: `https://github.com/chaitanya1chawla/dinov2-nist-detection`
- **Maintainer**: cchawla@andrew.cmu.edu

---

### LegoOffsetDetection
- **What it does**: Vision utility for detecting placement offsets/orientation of LEGO bricks to correct the end-effector/tool approach during assembly.
- **Where it’s used in AIDF**: Intended to provide live tool/placement offsets to the manipulation loop. In AIDF, a ROS subscriber listens to a tool offset topic (e.g., `/tool_offset`) for {x, y, theta} corrections, which can be sourced from this module.
- **Upstream**: `git@github.com:kevinPOI/LegoScope.git`
- **Maintainer**: cchawla@andrew.cmu.edu, zhenrant@andrew.cmu.edu
---

### lego-failure
- **What it does**: Analysis utilities to detect, classify, or log failure modes during LEGO assembly tasks (e.g., slips, misalignments, incomplete contacts). Useful for dataset curation and robustness studies.
- **Where it’s used in AIDF**: Complements experiments by diagnosing and visualizing failure cases. Not directly integrated into the core C++ planner; used as a sidecar tool during development/evaluation.
- **Upstream**: `git@github.com:mmliu2/lego-failure.git`
- **Maintainer**: yizhouhu@andrew.cmu.edu

---

### lego-failure-web
- **What it does**: A lightweight web interface for exploring or surfacing failure cases/results produced by the failure analysis tools.
- **Where it’s used in AIDF**: Optional front-end to review run logs, detections, and clips. Not required for the planner; useful during debugging and demos.
- **Upstream**: `git@github.com:mmliu2/lego-failure-web.git`
- **Maintainer**: yizhouhu@andrew.cmu.edu

---

### VideoSkillExtraction
- **What it does**: Processes human LEGO assembly videos (Gemini + segmentation utilities) to produce structured instructions and environment/task configuration JSONs.
- **Key entry points**: `gemini_test.py`, `scripts/` (video preprocessing and segmentation), `generated_configs/` (example outputs).
- **Where it’s used in AIDF**: Generates configuration artifacts (e.g., environment setup and assembly steps) intended to be compatible with the planning pipeline. These outputs can be adapted/merged into `config/lego_tasks/assembly_tasks/` or consumed by integration scripts.
- **Upstream**: `https://github.com/chaitanya1chawla/VideoSkillExtraction.git`
- **Maintainer**: cchawla@andrew.cmu.edu

---

### How these tools relate to the core planner
- The core planning/execution stack lives in `skillgraph/`, `planner/`, and `exe/`. The tools in this directory are complementary:
  - Perception/pose estimation (Dinov2-Nist-Detection, LegoOffsetDetection)
  - Failure analysis and visualization (lego-failure, lego-failure-web)
  - Data-to-config generation from videos (VideoSkillExtraction)

These tools are designed to feed data into the broader system via ROS topics or by generating JSON configs that the planner can use.

---

### Support and Contact
- **For issues with this AIDF integration**: Open an issue in the AIDF repository and include which tool you’re using and how it connects to your workflow.
- **For tool-specific questions/bugs**: Please use the upstream repositories’ issue trackers:
  - Dinov2-Nist-Detection: `https://github.com/chaitanya1chawla/dinov2-nist-detection`
  - LegoOffsetDetection (LegoScope): `https://github.com/kevinPOI/LegoScope` (use HTTPS if needed)
  - lego-failure: `https://github.com/mmliu2/lego-failure`
  - lego-failure-web: `https://github.com/mmliu2/lego-failure-web`
  - VideoSkillExtraction: `https://github.com/chaitanya1chawla/VideoSkillExtraction`

When reporting issues, include:
- A brief description of the task and expected behavior
- OS, Python/ROS versions (if applicable)
- Tool commit/version and command(s) used
- Relevant logs, screenshots, or small video clips if available


