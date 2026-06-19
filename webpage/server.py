from flask import Flask, request, jsonify, render_template, Response
import subprocess
import logging
from flask_cors import CORS
import json
import os
import signal
import time

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import threading
import base64
from pathlib import Path

app = Flask(__name__)
CORS(app)

WEBPAGE_DIR = Path(__file__).resolve().parent
REPO_ROOT = WEBPAGE_DIR.parent
CONFIG_DIR = REPO_ROOT / "config"
LOG_DIR = Path(os.environ.get("AIDF_WEB_LOG_DIR", "/tmp"))
SIMULATION_LOG = LOG_DIR / "aidf_simulation.log"
MOVEIT_LOG = LOG_DIR / "aidf_moveit.log"
REAL_ROBOT_LOG = LOG_DIR / "aidf_real_robot.log"

# Configure logging to log to terminal
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler()]
)

# Variable to keep track of the current simulation process
simulation_process = None

def stop_existing_simulation():
    global simulation_process

    if simulation_process is not None and simulation_process.poll() is None:
        logging.info(f"Stopping previous simulation process PID {simulation_process.pid}")
        simulation_process.terminate()
        try:
            simulation_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            simulation_process.kill()
            simulation_process.wait(timeout=5)

    simulation_process = None

    # Flask debug reloads can lose the Python Popen handle while the ROS process
    # keeps running, so also remove stale webplan_lego monitors.
    subprocess.run(
        ["pkill", "-f", r"webplan_lego .*config/web_message\.json"],
        check=False,
    )

# for demo use
def check_target(input_file, robot_id, obj, skill, target):

    with open(input_file, 'r') as file:
        data = json.load(file)
    # logging.info(f"input_file: {data}")
    for key, value in data.items():
        if value.get('object') == obj and value.get('skill') == skill and value.get('robot-id') == robot_id and \
            value.get('target-x') == target["x"] and value.get('target-y') == target["y"] and value.get('target-z') == target["z"] and value.get('ori') == target["ori"]:
            return key

    return None

def brick_id_from_object_name(object_name):
    if not object_name or not object_name.startswith("b") or "_" not in object_name:
        return None
    try:
        return int(object_name.split("_", 1)[0][1:])
    except ValueError:
        return None

def brick_seq_from_object_name(object_name):
    if not object_name or "_" not in object_name:
        return None
    try:
        return int(object_name.rsplit("_", 1)[1])
    except ValueError:
        return None

def find_matching_task_parameters(obj, target):
    skillgraph_path = CONFIG_DIR / "lego_tasks" / "skillgraph.json"
    if not skillgraph_path.exists():
        return {}

    with skillgraph_path.open("r") as file:
        skillgraph_json = json.load(file)

    assembly_seq = skillgraph_json.get("tasks", {}).get("assembly_seq")
    if not assembly_seq:
        return {}

    assembly_seq_path = REPO_ROOT / assembly_seq
    if not assembly_seq_path.exists():
        return {}

    brick_id = brick_id_from_object_name(obj)
    brick_seq = brick_seq_from_object_name(obj)
    if brick_id is None:
        return {}

    with assembly_seq_path.open("r") as file:
        task_json = json.load(file)

    fallback_match = {}
    for step in task_json.values():
        if int(step.get("brick_id", -1)) != brick_id:
            continue
        if int(step.get("x", -1)) != int(target.get("x", -1)):
            continue
        if int(step.get("y", -1)) != int(target.get("y", -1)):
            continue
        if int(step.get("z", -1)) != int(target.get("z", -1)):
            continue
        if int(step.get("ori", 0)) != int(target.get("ori", 0)):
            continue
        fallback_match = step
        if brick_seq is not None and int(step.get("brick_seq", brick_seq)) != brick_seq:
            continue
        return step

    return fallback_match

def check_simulation_log(command_id):
    if SIMULATION_LOG.exists():
        with SIMULATION_LOG.open('r') as log_file:
            for line in log_file:
                if f"error: not feasible, id: {command_id}" in line:
                    return line
    return False


# # Function to convert ROS Image message to OpenCV image
bridge = CvBridge()
latest_image = None
image_lock = threading.Lock()  # Add thread safety for image access
ros_initialized = False
moveit_process = None
task_planning_process = None

def vis_frame_callback(msg):
    """
    Callback function for processing image messages from the /vis_frame topic
    """
    global latest_image
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        with image_lock:
            latest_image = cv_image
    except CvBridgeError as e:
        logging.error(f"CV Bridge error: {str(e)}")

def ros_spin_thread():
    """
    Thread function that runs the ROS spin loop
    """
    rospy.spin()


# Initialize ROS node if it's not already initialized
def init_ros():
    global ros_initialized
    if not ros_initialized:
        try:
            # Initialize ROS node without a node name to avoid conflicts
            # with other nodes if roscore is already running
            rospy.init_node('flask_server', anonymous=True, disable_signals=True)
            
            # Check if our target topic exists
            vis_frame_exists = any(topic[0] == '/vis_frame' for topic in rospy.get_published_topics())
            
            # Subscribe to image topic
            rospy.Subscriber('/vis_frame', Image, vis_frame_callback)
            
            # Add a diagnostic timer to periodically check if we're receiving images
            def check_image_reception():
                global latest_image
                with image_lock:
                    has_image = latest_image is not None
                logging.info(f"Image reception check - Have we received an image? {has_image}")
                
                        # Start the ROS spin thread to process callbacks
            spin_thread = threading.Thread(target=ros_spin_thread)
            spin_thread.daemon = True  # Allow the thread to exit when main program exits
            spin_thread.start()
            logging.info("Started ROS spin thread")
                
            rospy.Timer(rospy.Duration(15), lambda event: check_image_reception())
            
            ros_initialized = True
        except Exception as e:
            logging.error(f"Failed to initialize ROS: {str(e)}")
            return False
    return True

    
@app.route('/start_simulator', methods=['POST'])
def start_simulator():
    global simulation_process

    logging.info("Starting Simulation...")
    stop_existing_simulation()
    data = request.json
    simulator = data.get("simulator")
    robot = data.get("robot")
    task = data.get("task") # change skillgraph.json

    # Modify these with your actual ROS package and launch file names
    ros_package = "your_ros_package" # TODO: Update the real ROS package name
    launch_file = "your_launch_file.launch" # TODO: Update the real launch file name

    if not task or task == "none":
        return jsonify({
            "status": -1,
            "output": "Please choose a task before starting the simulator."
        }), 400

    assembly_seq_path = CONFIG_DIR / "lego_tasks" / "assembly_tasks" / f"{task}.json"
    env_setup_path = CONFIG_DIR / "lego_tasks" / "env_setup" / f"env_setup_{task}.json"
    if not assembly_seq_path.exists():
        return jsonify({
            "status": -1,
            "output": f"Assembly task file not found: {assembly_seq_path}"
        }), 400
    if not env_setup_path.exists():
        return jsonify({
            "status": -1,
            "output": f"Environment setup file not found: {env_setup_path}"
        }), 400

    # change the skillgraph.json
    logging.info(f"task:{task}")
    skillgraph_path = CONFIG_DIR / "lego_tasks" / "skillgraph.json"
    with skillgraph_path.open('r') as file:
        skillgraph_json = json.load(file)
    skillgraph_json['tasks']['name'] = task
    skillgraph_json['tasks']['assembly_seq'] = f'config/lego_tasks/assembly_tasks/{task}.json'
    skillgraph_json['environment']['object_library'] = f'config/lego_tasks/env_setup/env_setup_{task}.json'
    with skillgraph_path.open('w') as file:
        json.dump(skillgraph_json, file, indent=4)
    logging.info(f"skillgraph.json updated successfully!")

    skillgraph_abspath = str(skillgraph_path.resolve())
    web_message_abspath = str((CONFIG_DIR / "web_message.json").resolve())
    logging.info(skillgraph_abspath)

    # Prepare the command to run the simulation
    command = ["rosrun", "aidf", "webplan_lego", skillgraph_abspath, web_message_abspath]
    # command = f'exec bash -i -c "conda deactivate && exec roslaunch robot_digital_twin dual_gp4.launch"'
    logging.info(f"Executing command: {command}")
    
    try:
        # Start the simulation in a subprocess
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        log_file = SIMULATION_LOG.open("w")
        env = os.environ.copy()
        env["AIDF_ROOT_DIR"] = str(REPO_ROOT)
        simulation_process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=log_file,
            cwd=str(REPO_ROOT),
            env=env
        )
        
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


@app.route('/run_simulation', methods=['POST'])
def run_simulation():
    # # Initialize ROS connection
    # global ros_initialized
    # if not ros_initialized:
    #     init_ros()

    global simulation_process

    logging.info("Running simulation...")
    data = request.json
    command_id = data.get("command_id")
    robot_id = data.get("robot_id")
    obj = data.get("object")
    skill = data.get("skill")
    target = data.get("target")
    skill_parameters = data.get("skill_parameters", {})  # Extract skill_parameters from the request

    # Update the web_message.json file with the extracted data
    web_message_path = CONFIG_DIR / "web_message.json"
    try:
        with web_message_path.open('r') as file:
            web_message = json.load(file)
        
        web_message['skill'] = skill
        web_message['object'] = obj
        web_message['robot'] = robot_id
        target_location = dict(target)
        task_parameters = find_matching_task_parameters(obj, target)
        if task_parameters:
            target_location.update(task_parameters)
        web_message['target_location'] = target_location
        web_message['skill_parameters'] = skill_parameters  # Update skill_parameters
        web_message['command_id'] = command_id

        with web_message_path.open('w') as file:
            json.dump(web_message, file, indent=4)
        
        logging.info(f"web_message.json updated successfully: {web_message}")
        
        # Check the simulation log for errors
        time.sleep(2)  # Wait for the simulation to run and log output
        check_simulation_log_result = check_simulation_log(command_id)
        if check_simulation_log_result:
            logging.error(f"Simulation error found in log: {check_simulation_log_result}")
            return jsonify({
                "status": -1,
                "output": check_simulation_log_result
            }), 400
            
        if skill == "Align":
            # TODO: Chaitanya - incorporate the folllowing command into the skill graph
            log_file = MOVEIT_LOG.open("w")
            # command = f'exec bash -i -c "exec roslaunch yk_launch moveit.launch namespace:=yk_destroyer"'
            # moveit_process = subprocess.Popen(command, stdout=log_file, stderr=log_file, shell=True)
            command = f'exec bash -i -c "exec roslaunch gp4_lego task_planning_chaitanya_node.launch namespace:=yk_destroyer"'
            task_planning_process = subprocess.Popen(command, stdout=log_file, stderr=log_file, shell=True)
            
            
            # get image from /vis_frame topic
            time.sleep(2)
            image = latest_image
            # convert image to base64
            with image_lock:
                image_base64 = cv2.imencode('.jpg', image)[1].tobytes()
                image_base64 = base64.b64encode(image_base64).decode('utf-8')
                
            # send image to the client
            return jsonify({
                "status": 0,
                "output": image_base64
            }), 200
            
        
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

@app.route('/run_real_robot', methods=['POST'])
def run_real_robot():
    global simulation_process

    logging.info("Running real robot...")
    data = request.json
    command_id = data.get("command_id")
    robot_id = data.get("robot_id")
    obj = data.get("object")
    skill = data.get("skill")
    target = data.get("target")
    skill_parameters = data.get("skill_parameters", {})  # Extract skill_parameters from the request

    if skill != "base":
        input_file = CONFIG_DIR / "general_tasks" / "processed_cliff_meta_skills.json"
        result = check_target(input_file, robot_id, obj, skill, target)
        if result == "9" or result == "10" or result == "11":
            num = result
        else:
            logging.error(f"Error found in log: error: not feasible {result}")
            return jsonify({
                "status": -1,
                "output": "error: not feasible, reason: target location is not feasible!"
            }), 400

        # send the ros command to the real robot
        command = f'roslaunch mr_planner mfi_lego.launch task:=cliff_{num}_{num}'
    else:
        command = f'roslaunch mr_planner mfi_lego.launch task:=base'

    logging.info(f"Executing command: {command}")
    
    try:
        # Start the simulation in a subprocess
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        log_file = REAL_ROBOT_LOG.open("w")
        simulation_process = subprocess.Popen(command, stdout=log_file, stderr=log_file, shell=True)
        
        logging.info(f"Real robot session started with PID {simulation_process.pid}")
        
        # Return success response with pid
        return jsonify({
            "status": 0,
            "output": "Real robot execution",
            "pid": simulation_process.pid
        }), 200
        
    except Exception as e:
        logging.error(f"Fail: {str(e)}")
        return jsonify({
            "status": -1,
            "error": str(e), 
            "output": "Failed to run real robot demo!"
            }), 500

@app.route('/add_skill', methods=['POST'])
def add_skill():
    logging.info("Adding skill...")
    data = request.json
    skill_name = data.get("skill_name")
    numRobot = data.get("num_robot")
    atomic_skills = data.get("atomic_skills")
    robot_id = data.get("robot_id")

    return jsonify({
            "status": 0,
            "output": f"Skill {skill_name} added successfully! with atomic skills: {atomic_skills}",
        }), 200

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
        
        # TODO: Chaitanya - killing the moveit and task planning processes
        if moveit_process is not None:
            moveit_process.kill()
            moveit_process = None
        if task_planning_process is not None:
            task_planning_process.kill()
            task_planning_process = None
        
        logging.info(f"Simulation with PID {pid} stopped.")
        return jsonify({"status": 0, "output": "Simulation stopped successfully"}), 200
    except Exception as e:
        logging.error(f"Exception occurred while stopping the simulation: {str(e)}")
        return jsonify({"error": str(e), "status": -1}), 500


default_image = None

# Add a streaming endpoint using Server-sent Events
@app.route('/start_camera_feed')
def start_camera_feed():
    global ros_initialized
    if not ros_initialized:
        init_ros()

    logging.info("Starting camera feed...")
    def generate():
        logging.info("Generating camera feed...")
        global default_image
        
        # Load default image if not already loaded
        if default_image is None:
            try:
                default_image_path = WEBPAGE_DIR / 'static' / 'images' / 'default_camera.jpg'
                if default_image_path.exists():
                    default_image = cv2.imread(str(default_image_path))
                else:
                    logging.warning(f"Default image not found at {default_image_path}")
                    # Create a simple black image with text as fallback
                    default_image = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(default_image, "No camera feed available", (50, 240), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            except Exception as e:
                logging.error(f"Error loading default image: {str(e)}")
                default_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(default_image, "Error loading camera feed", (50, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        yield "data: connected\n\n"  # Initial connection message
        while True:
            global latest_image
            image_to_send = None
            
            if latest_image is not None:
                with image_lock:
                    image_to_send = latest_image.copy()
            else:
                image_to_send = default_image
            
            # Convert image to base64
            _, buffer = cv2.imencode('.jpg', image_to_send, [cv2.IMWRITE_JPEG_QUALITY, 80])
            img_str = base64.b64encode(buffer).decode('utf-8')
            
            yield f"data: {img_str}\n\n"
            time.sleep(0.1)  # 10 FPS
    
    return Response(generate(), mimetype="text/event-stream")

if __name__ == '__main__':
    try:
        # Start the Flask server
        app.run(host='0.0.0.0', port=5000, debug=True)
    except KeyboardInterrupt:
        logging.info("Server shutdown requested...")
    finally:
        # Cleanup on exit
        # No need for global declarations here - variables are already in module scope
        streaming_active = False
        if simulation_process:
            simulation_process.kill()
        # if moveit_process:
        #     moveit_process.kill()
        # if task_planning_process:
        #     task_planning_process.kill()
        
        logging.info("Server shut down successfully")
