# AIDF

[![Project Page](https://img.shields.io/badge/Project-Website-2ea44f?logo=githubpages&logoColor=white)](https://intelligent-control-lab.github.io/AIDF/) [![arXiv](https://img.shields.io/badge/arXiv-2603.12649-b31b1b?logo=arxiv&logoColor=white)](https://arxiv.org/pdf/2603.12649) [![Code](https://img.shields.io/badge/Code-GitHub-181717?logo=github&logoColor=white)](https://github.com/intelligent-control-lab/AIDF)

This is the code repository repo for AIDF project. The goal of this repo is to provide the software implementation for the skillgraph and planner, based on our ontological layout. 

## 📦 Build Instruction For ROS1
Download and build the codebase using standard ROS1 ```catkin_tools```
To enable our ROS Noetic backend (tested on Ubuntu 20.04), you may need to install some system dependencies
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [moveit](https://moveit.ai/install/)
- [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
- [rviz tools](http://wiki.ros.org/rviz_visual_tools)
- [moveit visual tools](http://wiki.ros.org/moveit_visual_tools)

For example, run
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - &&
sudo apt update &&
sudo apt install ros-noetic-desktop-full
```

to install ROS 1 noetic, then run
```
sudo apt install ros-noetic-rviz ros-noetic-moveit python3-catkin-tools ros-noetic-rviz-visual-tools ros-noetic-moveit-visual-tools
```
to install moveit, rviz and catkin tools

Under your ```catkin_ws/src```, download the GP4-Lego simulator and environment setup to your workspace
- [gp4 digital twin](https://github.com/intelligent-control-lab/Robot_Digital_Twin.git). checkout to the ``dual_arm_gen3`` branch!
- this repo

For example, run
```
mkdir -p ~/catkin_ws/src ; cd ~/catkin_ws ; catkin init
```
to initialize catkin workspace.

Run
```
cd ~/catkin_ws/src ; git clone https://github.com/intelligent-control-lab/Robot_Digital_Twin.git ; cd Robot_Digital_twin ; git checkout dual_arm_gen3
cd ~/catkin_ws/src ; git clone https://github.com/intelligent-control-lab/AIDF.git
```

To build the repo, run
```
cd ~/catkin_ws/src ; catkin build 
```

## Docker Workflow
The Docker image contains the ROS1 dependencies used by the Lego simulator and webpage API.

### Build the Image
If the repository or dependencies require private GitHub access, export a GitHub token before building:

```bash
export GITHUB_TOKEN="your_github_token_here"
cd docker
./build.sh
cd ..
```

### Launch a Shell Container
Use this when you want an interactive ROS terminal inside Docker:

```bash
./docker/launch.sh
```

The container is named `aidf_container`. To open another shell in the same running container:

```bash
docker exec -it aidf_container bash
```

### Launch the Webpage
Use web mode to start both the Flask backend and static frontend:

```bash
./docker/launch.sh --web
```

This mounts the local repo into `/root/catkin_ws/src/AIDF`, rebuilds the `aidf` package, then starts:

| Service | URL |
| --- | --- |
| Webpage | http://localhost:8000/webpage/api.html |
| Backend API | http://localhost:5000 |

Logs are written inside the container to:

```text
/tmp/aidf_web_backend.log
/tmp/aidf_web_frontend.log
/tmp/aidf_simulation.log
```

Stop web mode with `Ctrl+C` in the launch terminal.

If no ROS master is already running, open a second terminal and start one inside the running container:

```bash
docker exec -it aidf_container bash
source /opt/ros/noetic/setup.bash
roscore
```

Keep that terminal open while using the simulator.

## Run Lego Assembly Simulation
AIDF supports two execution paths:

- Planner execution: automatically searches for and runs a feasible skill sequence.
- Web execution: sends individual skills through the webpage API.

### Planner Execution
Inside the Docker shell, run:

```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
rosrun aidf plan_lego
```

You should see the planner generate a skill sequence and execute it in RViz.

### Web Execution
1. Start the webpage:

   ```bash
   ./docker/launch.sh --web
   ```

2. Open http://localhost:8000/webpage/api.html.
3. In **Simulator Controls**, choose:

   | Field | Value |
   | --- | --- |
   | Simulator | Usually `Moveit` |
   | Robot | Usually `gp4` |
   | Task | Any task shown in the dropdown |

4. Click **Start Simulator**.
5. Run skills manually from the webpage, or run the automated replay script below.

## Manual Web Test
The webpage sends one meta-skill command at a time to the simulator.

1. Start a simulator from **Simulator Controls**.
2. Choose a **MetaSkill**, **Object**, **Primary Robot**, and target pose.
3. Leave **Skill Parameters** empty for normal Lego assembly tests. The backend fills task-specific fields such as press side, support pose, and handover type from the selected task JSON.
4. Click **Run Simulation**.
5. Watch RViz. If the command is feasible, the robot executes the skill. If not, the webpage shows the feasibility error returned by the skillgraph.

Use these files as the source of truth for manual test values:

| File | What to read |
| --- | --- |
| `config/lego_tasks/assembly_tasks/<task>.json` | Assembly step order, target `x/y/z/ori`, support fields, handover flags |
| `config/lego_tasks/env_setup/env_setup_<task>.json` | Available object names for the task |

For example, an assembly row with `support_x != -1` should be tested with `PickAndPlaceWithSupport`. A row with `manipulate_type == 1` should be tested with `PickHandoverAndPlace`. Otherwise, use `PickAndPlace`.

To generate a click-by-click sequence for a task, run a dry-run:

```bash
python3 webpage/test_web_skill_sequence.py --task <task_name> --dry-run
```

Example command types you may see:

| Task row condition | Web skill to choose |
| --- | --- |
| `support_x == -1` and `manipulate_type == 0` | `PickAndPlace` |
| `support_x != -1` | `PickAndPlaceWithSupport` |
| `manipulate_type == 1` | `PickHandoverAndPlace` |

After a skill succeeds, its object is removed from the webpage dropdown to avoid accidental reuse in one-off manual tests.

## Automatic Web Test
The script [webpage/test_web_skill_sequence.py](/home/patricia/Desktop/Learn/codes/AIDF/webpage/test_web_skill_sequence.py) sends the same skill commands that the webpage sends to the backend API.

Start the web services first:

```bash
./docker/launch.sh --web
```

List tasks that can be derived automatically:

```bash
python3 webpage/test_web_skill_sequence.py --list-tasks
```

Run one task by name:

```bash
python3 webpage/test_web_skill_sequence.py --task <task_name> --start-simulator --fail-fast --delay 5 --timeout 60
```

Example:

```bash
python3 webpage/test_web_skill_sequence.py --task faucet --start-simulator --fail-fast --delay 5 --timeout 60
```

Useful variants:

```bash
# Print the generated sequence without sending commands.
python3 webpage/test_web_skill_sequence.py --task <task_name> --dry-run

# Reuse a simulator already started from the webpage.
python3 webpage/test_web_skill_sequence.py --task <task_name> --fail-fast --delay 5 --timeout 60

# Replay a planner-generated skillplan instead of deriving from a task.
python3 webpage/test_web_skill_sequence.py --skillplan config/lego_tasks/skillplan.json --start-simulator --fail-fast

# Replay a custom ordered command sequence.
python3 webpage/test_web_skill_sequence.py --sequence path/to/sequence.json --sim-task <task_name> --start-simulator
```

Notes:

- `--task <task_name>` derives commands from `assembly_tasks/<task_name>.json` and `env_setup/env_setup_<task_name>.json`.
- Some large tasks use reusable source stations, so the derived command sequence may reuse object names for repeated brick types.
- If no source argument is given, the default replay source is `config/lego_tasks/skillplan.json`.
- Increase `--delay` if you want more time to watch each motion in RViz.
- Use `--fail-fast` while debugging so the script stops at the first infeasible skill.
- `TranslateWithRotation` requires optional `yk_tasks` support; the default Docker build may skip it if those services are not available.

## 📁 File Structure
```text
AIDF/
├── config/
│   ├── ddb/                         # Digital data backbone configuration
│   ├── general_tasks/               # Generic task examples
│   └── lego_tasks/
│       ├── assembly_tasks/          # Task-level Lego assembly JSON files
│       ├── env_setup/               # Initial object layouts for each task
│       ├── robot_properties/        # Robot calibration and DH files
│       ├── steps/                   # Precomputed task step and sequence files
│       ├── skillgraph.json          # Skillgraph and metaskill configuration
│       └── skillplan.json           # Planner-generated replay example
├── docker/
│   ├── Dockerfile                   # ROS1/MoveIt Docker image
│   ├── build.sh                     # Image build helper
│   ├── launch.sh                    # Shell or web-mode container launcher
│   └── start_web_services.sh        # Backend/frontend startup for web mode
├── docs/                            # Additional setup and design notes
├── exe/
│   ├── plan_lego.cpp                # Planner execution entry point
│   └── webplan_lego.cpp             # Web-controlled simulation entry point
├── executor/                        # Python execution abstraction and backends
├── launch/                          # ROS launch files
├── planner/
│   ├── include/                     # Planner headers
│   └── src/                         # Planner implementation
├── skillgraph/
│   ├── api/                         # Core C++ class and data structure headers
│   ├── include/                     # Backend and algorithm headers
│   └── src/                         # Skillgraph, MoveIt backend, and Lego skills
├── srv/                             # ROS service definitions
├── tools/                           # Perception, detection, and failure-analysis tools
├── webpage/
│   ├── api.html                     # Browser UI
│   ├── server.py                    # Flask API bridge to ROS/C++ simulation
│   └── test_web_skill_sequence.py   # Automated webpage-command replay script
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 🚀 Current Implementation Status
Implemented:

- [x] ROS1/MoveIt Lego simulation with Docker support
- [x] C++ skillgraph, Lego configs, and planner executable
- [x] Web UI, Flask API bridge, and automated skill replay

Not yet implemented:

- [ ] ROS2 backend integration
- [ ] Other simulations: e.g. Gazebo and Isaac Sim
- [ ] Other robot: e.g. Fanuc

<!-- ### Contributors
Philip Huang

Peiqi Yu -->
