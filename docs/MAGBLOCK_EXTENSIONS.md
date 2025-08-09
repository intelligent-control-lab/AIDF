# Magblock Extensions to the SkillGraph Framework

This document describes the **Magnetic Block extensions** to the Skill Graph framework in the `AIDF` repository.
These extensions introduce new **domain-specific skills and objects** for magnetic block assembly, enabling multi-robot pick and place tasks with placement constraints and MoveIt2 motion planning integration.

## Overview
The Magblock integration expands the existing Skill Graph architecture with:
- Domain specific task descriptions (assembly/environment/skillgraph files)
- Magblock-specific object and skill definitions
- MoveIt2 backend support for simulation and trajectory generation
- Reference frame conventions for consisted block alignment and attachment

## New Components
The following files have been added under the magnetic block integration branch:

| File | Purpose |
|------|---------|
| `mag_block_tasks/` | Contains **assembly task JSON files**, environment descriptions, and skillgraph configuration for MagBlock assemblies. |
| `magblock_algorithms.hpp/cpp` | Algorithms for handling MagBlock-specific planning, constraints, and press-face alignment. |
| `magblock_objects.hpp/cpp` | Definitions for MagBlock objects, including geometry, magnetic polarity, and grasp points. |
| `magblock_skillgraph.hpp/cpp` | MagBlock-specific SkillGraph logic, connecting symbolic skill descriptions to MoveIt 2 motion planning. |
| `magblock_skills.hpp/cpp` | Implementation of MagBlock-specific skills (`Pick`, `Place`, `Transit`, `PickAndPlace`) with domain parameters. |

---

## Magblock Skills
Four new skills have been implemented for Magblock assembly:
### 1. `Pick`
- Grasps a magnetic block from its current location.
- Ensures **correct face orientation** for joining.

### 2. `Place`
- Aligns and attaches the held block to the assembly structure.
- Uses `press_face` to identify **which block face** makes contact with the structure.
- Executes press motion to ensure proper magnetic connection.

### 3. `Transit`
- Moves the robot from one location to another without manipulating objects.

### 4. `PickAndPlace`
- Combined pick and place action for efficiency.
- Executes as a **MetaSkill** using `Pick` + `Transit` + `Place`.

--- 


## Domain Parameters

Two new parameters were introduced to capture MagBlock assembly constraints:

| Parameter | Description |
|-----------|-------------|
| `press_face` | The face of the MagBlock to be pressed against the target structure during placement. |
| `gripper_ori` | The required orientation of the gripper/end effector to achieve correct block alignment. |

---

## Reference Frame Convention

## MoveIt 2 Backend:
The MagBlock integration uses **MoveIt 2** for:
- Inverse kinematics
- Trajectory planning
- Collision checking
- Object visualization during execution

---

## Launching Simulations

Two launch files are provided for testing MagBlock assemblies in simulation:

```bash
# Standard assembly execution
ros2 launch aidf magblock_assembly_test.launch.py
```

```bash
# Temporal Plan Graph (TPG)-based coordinated execution
ros2 launch aidf magblock_tpg.launch.py
```

## Example Workflow
1. Prepare the environment

   Modify or create an environment file in mag_block_tasks/envs/.
2. Define an assembly task
   
    Create a JSON assembly description in mag_block_tasks/assemblies/ with press_face and gripper_ori defined for each step.
3. Launch simulation
   
    Run one of the provided launch files to start the MoveIt 2-based simulation.
4. Observe skill execution
   
   Robots will perform pick/place operations using the MagBlock-specific skills.
