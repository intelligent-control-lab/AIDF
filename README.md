# AIDF

This is the code repository repo for AIDF project. The goal of this repo is to provide the software implementation for the skillgraph and planner, based on our ontological layout. 

### Build Instruction
Download and build the codebase using standard ROS1 ```catkin_tools```


### File Structure
/AIDF <br>
│─ config/<br>
│─ planner/<br>
│─ skillgraph/<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ api/ (class and data structure definitions)<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ Utils/<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ skillgraph.hpp (implements the structure of the skill graph)<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ algorithms.hpp (implements SkillExecutor and Algorithm)<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ backend.hpp (implements the backend definitions in Environment)<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ environment.hpp (implements Environment)<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ metrics.hpp (implements Metric)<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ objects.hpp (implements Object)<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ robots.hpp (implements Robot)<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ skills.hpp (implements Skill)<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ tasks.hpp (implements TaskParam)<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ data_structure.hpp (implements Data Structures)<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ src/<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ corresponding cpp files...<br>
│─ include/<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ lego/ (lego-specific implementations)<br>
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│─ Instance Implementation.hpp files<br>


### Current Implementation Status
- [x] Class Definition in Header Files
- [ ] Lego Skills
- [ ] Integration with ROS2 backend
- [ ] Naive Planner
- [ ] Controller

<!-- ### Contributors
Philip Huang

Peiqi Yu -->