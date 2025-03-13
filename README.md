# AIDF

This is the code repository repo for AIDF project. The goal of this repo is to provide the software implementation for the skillgraph and planner, based on our ontological layout. 

## 📦 Build Instruction
Download and build the codebase using standard ROS1 ```catkin_tools```

## 🚀 Webpage On The Way!
### What's its function?
The webpage [webpage](api.html) interacts with the simulators through communication with the [server](server.py).

### How to run the webpage?
Start a terminal, run ```python server.py```, and start another terminal, run ```python -m http.server```, then go to http://127.0.0.1:8000/api.html, and you would see the webpage! 

### How to interact with the simulator?
After you opened up the webpage and the server, choose your preferred simulator, robot type, and task under **[Simulator Controls]**, and then click on **[Start Simulator]**. If the **[Response Received from Server]** shows the message is correctly delivered, you should see you simulator start to run( make sure you have your simulator correctly installed and the environment approriately activated!). <br>

After the simulation has been started, choose skill, object, main robot, and target locations, and then click on **[Run Target Task]**. If your chosen set is feasible, you would see the robot moving as expected; otherwise, you would see an error pop out.

### How to modify the commands?
Given that everyone's execution environment differs, we encourage you to check and modify the command ```command = xxx # TODO: Update the real execution commands``` in [server.py](server.py) in order for successful interaction. 

## 📁 File Structure
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

## 🚀 Current Implementation Status
- [x] Class Definition in Header Files
- [ ] Lego Skills
- [ ] Integration with ROS2 backend
- [ ] Naive Planner
- [ ] Controller

<!-- ### Contributors
Philip Huang

Peiqi Yu -->