# AIDF

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

## Run Lego Assembly Simulation
We have two ways to run the Lego Assembly simulation - with either of the following
- automacially plan and execute a sequence of skills for an assembly sequence, with the automated task planner 
- manually execute skills via an interactive webpage

### Automated Eecution with Planner
To run the automated planner, run
```
source /opt/ros/noetic/setup.bash ; source ~/catkin_ws/devel/setup.bash ; 
rosrun aidf plan_lego
```
You should see the planner automatically search for and generate a feasible skills sequence, and robot execution in an Rviz window.

### Manual Execution via Webpage

To start the frontend of the webpage, run
```
run_frontend.sh
```
and visit [localhost:8000/api.html](http://localhost:8000/api.html) for the web visualization.

To start the backend of the webpage (that connects to the C++ skillgraph implementation), run
```
run_backend.sh
```


## 🚀 Webpage On The Way!
### What's its function?
The webpage [api.html](api.html) interacts with the simulators through communication with the [server](server.py).

### How to run the webpage?
Under the AIDF repo, start a terminal, run ```python server.py```, and start another terminal, run ```python -m http.server```, then go to http://127.0.0.1:8000/api.html, and you would see the webpage! 

### How to interact with the simulator?
After you opened up the webpage and the server, choose your preferred simulator, robot type, and task under *"Simulator Controls"*, and then click on **[Start Simulator]**. If the *"Response Received from Server"* shows the message is correctly delivered, you should see you simulator start to run( make sure you have your simulator correctly installed and the environment approriately activated!). <br>

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
- [x] Basic web interface
- [ ] Lego Skills
- [ ] Integration with ROS2 backend
- [ ] Naive Planner
- [ ] Controller

<!-- ### Contributors
Philip Huang

Peiqi Yu -->