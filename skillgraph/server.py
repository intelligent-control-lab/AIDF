from flask import Flask, request, jsonify
import subprocess
import logging
from flask_cors import CORS
import os
import signal
import time

app = Flask(__name__)
CORS(app)

# Configure logging to log to terminal
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler()]
)

# Variable to keep track of the current simulation process
simulation_process = None

@app.route('/start_simulator', methods=['POST'])
def start_simulator():
    global simulation_process

    logging.info("Starting Simulation...")
    data = request.json
    simulator = data.get("simulator")
    robot = data.get("robot")
    task = data.get("task")

    # Modify these with your actual ROS package and launch file names
    ros_package = "your_ros_package"
    launch_file = "your_launch_file.launch"

    # Prepare the command to run the simulation
    command = f'exec bash -i -c "conda deactivate && exec roslaunch {ros_package} {launch_file} \
        simulator:={simulator} robot:={robot} task:={task}"'
    command = f'exec bash -i -c "conda deactivate && exec roslaunch robot_digital_twin dual_gp4.launch"'
    logging.info(f"Executing command: {command}")
    
    try:
        # Start the simulation in a subprocess
        simulation_process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        
        logging.info(f"Simulation started with PID {simulation_process.pid}")
        
        # Return success response with pid
        return jsonify({
            "status": 0,
            "output": "Simulation started successfully",
            "pid": simulation_process.pid
        }), 200
    except Exception as e:
        logging.error(f"Exception occurred: {str(e)}")
        return jsonify({"error": str(e), "status": -1}), 500

@app.route('/run_target_task', methods=['POST'])
def run_target_task():
    global simulation_process

    logging.info("Starting Simulation...")
    data = request.json
    simulator = data.get("simulator")
    robot = data.get("robot")
    obj = data.get("object")
    skill = data.get("skill")
    target = data.get("target")

    # Modify these with your actual ROS package and launch file names
    ros_package = "your_ros_package"

    # Prepare the command to run the simulation
    command = f'exec bash -i -c "echo {ros_package} \
        simulator={simulator} robot={robot} object={obj} skill={skill} target={target}"'
    # command = f'exec bash -i -c "conda deactivate && exec roslaunch robot_digital_twin dual_gp4.launch"'
    logging.info(f"Executing command: {command}")
    
    try:
        # Start the simulation in a subprocess
        # pre_process = subprocess.run(command1, shell=True)
        simulation_process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        
        logging.info(f"Runing task with simulation {simulation_process.pid}")
        
        # Return success response with pid
        return jsonify({
            "status": 0,
            "output": "Task Executed successfully",
            "pid": simulation_process.pid
        }), 200
    except Exception as e:
        logging.error(f"Exception occurred: {str(e)}")
        return jsonify({"error": str(e), "status": -1}), 500


@app.route('/stop_simulation', methods=['POST'])
def stop_simulation():
    global simulation_process

    pid = request.json.get("pid")
    if pid is None or simulation_process is None or simulation_process.pid != pid:
        return jsonify({"error": "No running simulation found with the provided pid."}), 400
    
    try:
        # Send SIGTERM to the process to stop the simulation
        # os.killpg(os.getpgid(simulation_process.pid), signal.SIGTERM)
        simulation_process.kill()
        simulation_process = None
        logging.info(f"Simulation with PID {pid} stopped.")
        return jsonify({"status": 0, "output": "Simulation stopped successfully"}), 200
    except Exception as e:
        logging.error(f"Exception occurred while stopping the simulation: {str(e)}")
        return jsonify({"error": str(e), "status": -1}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
