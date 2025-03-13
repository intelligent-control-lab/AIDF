from flask import Flask, request, jsonify
import subprocess
import logging
from flask_cors import CORS
import json
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
    task = data.get("task") # change skillgraph.json

    # Modify these with your actual ROS package and launch file names
    ros_package = "your_ros_package" # TODO: Update the real ROS package name
    launch_file = "your_launch_file.launch" # TODO: Update the real launch file name

    # Prepare the command to run the simulation
    command = f'exec bash -i -c "exec rosrun aidf webplan_lego"' # TODO: Update the real execution commands
    # command = f'exec bash -i -c "conda deactivate && exec roslaunch robot_digital_twin dual_gp4.launch"'
    logging.info(f"Executing command: {command}")
    
    try:
        # Start the simulation in a subprocess
        log_file = open("simulation.log", "w")
        simulation_process = subprocess.Popen(command, stdout=log_file, stderr=log_file, shell=True)
        
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
    robot_id = data.get("robot_id")
    obj = data.get("object")
    skill = data.get("skill")
    target = data.get("target")

    # Update the web_message.json file with the extracted data
    web_message_path = './config/web_message.json'
    try:
        with open(web_message_path, 'r') as file:
            web_message = json.load(file)
        
        web_message['skill'] = skill
        web_message['object'] = obj
        web_message['robot'] = robot_id
        web_message['target_location'] = target

        with open(web_message_path, 'w') as file:
            json.dump(web_message, file, indent=4)
        
        logging.info(f"web_message.json updated successfully: {web_message}")
        return jsonify({
            "status": 0,
            "output": f"web_message.json updated successfully: {web_message}",
        }), 200
    except Exception as e:
        logging.error(f"Failed to update web_message.json: {str(e)}")
        return jsonify({
            "status": -1,
            "error": str(e), 
            "output": "Failed to update web_message.json"
            }), 500


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
