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

def check_simulation_log(command_id):
    log_file_path = "simulation.log"
    if os.path.exists(log_file_path):
        with open(log_file_path, 'r') as log_file:
            for line in log_file:
                if f"error: not feasible, id: {command_id}" in line:
                    return line
    return False


# Function to convert ROS Image message to OpenCV image
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
    print(f"Received image from /vis_frame callback")
    global latest_image
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        with image_lock:
            latest_image = cv_image
        logging.info("Received new image from /vis_frame")
        print('Received new image from /vis_frame')
    except CvBridgeError as e:
        print(f"CV Bridge error: {str(e)}")
        logging.error(f"CV Bridge error: {str(e)}")

def ros_spin_thread():
    """
    Thread function that runs the ROS spin loop
    """
    print("Starting ROS spin loop")
    rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    rospy.spin()
    #     time.sleep(0.01)
    #     rate.sleep()


# Initialize ROS node if it's not already initialized
def init_ros():
    global ros_initialized
    if not ros_initialized:
        try:
            # Initialize ROS node without a node name to avoid conflicts
            # with other nodes if roscore is already running
            rospy.init_node('flask_server', anonymous=True, disable_signals=True)
            
            # Check ROS master connection
            if not rospy.is_shutdown():
                print("Successfully connected to ROS master")
            else:
                print("Failed to connect to ROS master")
                return False
            
            # Check if our target topic exists
            vis_frame_exists = any(topic[0] == '/vis_frame' for topic in rospy.get_published_topics())
            print(f"'/vis_frame' topic exists: {vis_frame_exists}")
            print('checked')
            
            # Subscribe to image topic
            rospy.Subscriber('/vis_frame', Image, vis_frame_callback)
            print('checked')

            
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
            
            print("ROS node initialized and subscribed to /vis_frame")
            ros_initialized = True
        except Exception as e:
            logging.error(f"Failed to initialize ROS: {str(e)}")
            return False
    else:
        print('ROS node already initialized')
    return True

    
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

    # change the skillgraph.json
    logging.info(f"task:{task}")
    skillgraph_path = './config/lego_tasks/skillgraph.json'
    with open(skillgraph_path, 'r') as file:
        skillgraph_json = json.load(file)
    skillgraph_json['tasks']['name'] = task
    skillgraph_json['tasks']['assembly_seq'] = f'config/lego_tasks/assembly_tasks/{task}.json'
    with open(skillgraph_path, 'w') as file:
        json.dump(skillgraph_json, file, indent=4)
    logging.info(f"skillgraph.json updated successfully!")

    # Prepare the command to run the simulation
    # command = f'exec bash -i -c "echo "test"'
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
    
    # Initialize ROS connection
    global ros_initialized
    if not ros_initialized:
        init_ros()
        
    global simulation_process

    logging.info("Running target task...")
    data = request.json
    command_id = data.get("command_id")
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
        web_message['command_id'] = command_id

        with open(web_message_path, 'w') as file:
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
            
        if skill == "PickAndPlaceRealRobot":
            # TODO: Chaitanya - incorporate the folllowing command into the skill graph
            log_file = open("moveit.log", "w")
            command = f'exec bash -i -c "exec roslaunch yk_launch moveit.launch namespace:=yk_destroyer"'
            moveit_process = subprocess.Popen(command, stdout=log_file, stderr=log_file, shell=True)
            command = f'exec bash -i -c "exec roslaunch gp4_lego task_planning_chaitanya_node.launch namespace:=yk_destroyer"'
            task_planning_process = subprocess.Popen(command, stdout=log_file, stderr=log_file, shell=True)
            
            
            # # get image from /vis_frame topic
            # time.sleep(2)
            # image = latest_image
            # # convert image to base64
            # with image_lock:
            #     image_base64 = cv2.imencode('.jpg', image)[1].tobytes()
            #     image_base64 = base64.b64encode(image_base64).decode('utf-8')
                
            # # send image to the client
            # return jsonify({
            #     "status": 0,
            #     "output": image_base64
            # }), 200
            
        
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
                default_image_path = os.path.join('static', 'images', 'default_camera.jpg')
                if os.path.exists(default_image_path):
                    default_image = cv2.imread(default_image_path)
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
                print('Sending latest image')
            else:
                # print('No latest image available')
                # Use default image if latest_image is None
                image_to_send = default_image
                print("Using default image")
            
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
        if moveit_process:
            moveit_process.kill()
        if task_planning_process:
            task_planning_process.kill()
        
        logging.info("Server shut down successfully")
