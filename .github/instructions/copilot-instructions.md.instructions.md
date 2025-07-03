---
applyTo: '**'
---
Coding standards, domain knowledge, and preferences that AI should follow.

This is a ROS package that implements an ontology for robot skill graph.
It is designed to be compatiable with both ROS 1 and ROS 2, but the current repo is in a ROS 2 workspace,
Hence you can build the code with colcon build. Check the CMakelist.txt for build instructions

The existing codebase was tailored to LEGO assembly in ROS1, but the skeleton of skillgraph should be domain agnostic.
We are working on expanding the codebase to Mangietc Block assembly with multiple Kinova Gen3 arms in ROS2.

The skillgraph directory contains the main api and implementation of our "skillgraph". The domain independent interfaces are located in the skillgraph/api directory.
Domain dependent interfaces are located in the skillgraph/include directory.
Implementation are located in the skillgraph/src directory.

The configuration of the skilgraph should be located in the config directory. We use json to store human-readable information about the skillgraph in this directory.
Usually, the json should contain information about robots, environments, skills, and tasks.
