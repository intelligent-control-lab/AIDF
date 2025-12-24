# SkillPlan Executor (M2)

This is a lightweight Python executor/validator for `aidf.skillplan` JSON files produced by the skillgraph planner. Given a general, self-contained JSON file that a planned sequence of robot skills, it is able to execute robot skills sequentially in simulation and on real robot using ROS.

## Usage

From `catkin_ws/src/AIDF`:

```bash
python3 -m executor --plan config/lego_tasks/skillplan.json --backend dryrun
python3 -m executor --plan config/lego_tasks/skillplan.json --backend mujoco
```

Real-robot (ROS service) backend:

```bash
# Requires a sourced ROS env and the yk services running.
python3 -m executor \
  --plan config/lego_tasks/skillplan.json \
  --backend ros \
  --ros-service left_arm=/yk_destroyer/yk_execute_trajectory \
  --ros-service right_arm=/yk_architect/yk_execute_trajectory
```

MuJoCo backend options:

```bash
python3 -m executor \
  --plan config/lego_tasks/skillplan.json \
  --backend mujoco \
  --model ../robot_digital_twin/gazebo/urdf/dual_gp4.urdf.xacro \
  --viewer --realtime --dt 0.01
```

Notes:
- MuJoCo 3.x can load URDF directly; if `--model` is a `.xacro`, the backend tries to expand it via `rosrun xacro xacro` and falls back to the sibling `.urdf` when available.
- Playback is kinematic (joint positions are applied directly from trajectory points).
