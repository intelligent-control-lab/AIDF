#!/usr/bin/env python3

"""
ROS python node to connect to a rosbridge server, subscribe to two camera topics and do online inference using the geometric Lego state inferer.
"""
import asyncio
import websockets
import json
import cv2
import numpy as np
import base64
from pathlib import Path
import time

# Assuming OnlineLegoInferer is in a module that can be imported
from lego_online_infer import OnlineLegoInferer # Ensure this path is correct for your Docker env
import lego_visualize

# Helper to convert OpenCV image to ROS Image JSON for rosbridge
def cv_image_to_ros_image_json(cv_image, encoding, frame_id="camera", timestamp=None):
    if timestamp is None:
        timestamp = time.time()
    
    height, width = cv_image.shape[:2]
    is_color = len(cv_image.shape) == 3 and cv_image.shape[2] == 3
    is_rgba = len(cv_image.shape) == 3 and cv_image.shape[2] == 4

    if encoding == "rgb8" and is_color:
        pass # Already RGB
    elif encoding == "bgr8" and is_color:
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) # rosbridge might expect RGB for rgb8
    elif encoding == "rgba8" and is_rgba:
        pass # Already RGBA
    elif encoding == "mono8" and len(cv_image.shape) == 2:
        pass
    else:
        # Add more conversions if needed or raise error
        print(f"Warning: Unsupported encoding conversion for publishing: {encoding} from image shape {cv_image.shape}")
        # Fallback: try to encode as is, rosbridge might handle it or error
    
    image_data_bytes = cv_image.tobytes()
    image_data_b64 = base64.b64encode(image_data_bytes).decode('utf-8')

    msg = {
        "header": {
            "stamp": {"secs": int(timestamp), "nsecs": int((timestamp % 1) * 1e9)},
            "frame_id": frame_id
        },
        "height": height,
        "width": width,
        "encoding": encoding,
        "is_bigendian": 0,
        "step": width * cv_image.shape[2] if len(cv_image.shape) == 3 else width, # width * num_channels
        "data": image_data_b64
    }
    return msg

# Helper to convert ROS Image JSON from rosbridge to OpenCV image
def ros_image_json_to_cv_image(json_msg):
    print(list(json_msg.keys())) # Debugging line to check keys
    encoding = json_msg['encoding']
    height = json_msg['height']
    width = json_msg['width']
    step = json_msg['step']
    data_b64 = json_msg['data']
    
    image_data = base64.b64decode(data_b64)
    
    # Determine expected channels and dtype from encoding
    if encoding == 'rgb8':
        dtype = np.uint8
        channels = 3
    elif encoding == 'bgr8':
        dtype = np.uint8
        channels = 3
    elif encoding == 'rgba8':
        dtype = np.uint8
        channels = 4
    elif encoding == 'bgra8':
        dtype = np.uint8
        channels = 4
    elif encoding == 'mono8':
        dtype = np.uint8
        channels = 1
    elif encoding == 'mono16':
        dtype = np.uint16
        channels = 1
    else:
        raise ValueError(f"Unsupported image encoding: {encoding}")

    if channels == 1:
        cv_image = np.frombuffer(image_data, dtype=dtype).reshape((height, width))
    else:
        cv_image = np.frombuffer(image_data, dtype=dtype).reshape((height, width, channels))

    # OpenCV typically uses BGR by default for color images
    if encoding == 'rgb8':
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
    elif encoding == 'rgba8':
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGRA) # Or to BGR if alpha not needed downstream
    
    return cv_image

def ros_compressed_image_json_to_cv_image(json_compressed_msg):
    """
    Converts a ROS CompressedImage JSON message (from rosbridge) to an OpenCV image.
    """
    # json_compressed_msg is the 'msg' field from the rosbridge message
    # It should contain 'format' (e.g., 'jpeg', 'png') and 'data' (base64 encoded)
    img_format = json_compressed_msg['format']
    data_b64 = json_compressed_msg['data']
    
    compressed_data = base64.b64decode(data_b64)
    
    # Convert binary data to NumPy array
    np_arr = np.frombuffer(compressed_data, np.uint8)
    
    # Decode image using OpenCV
    # cv2.imdecode reads an image from a buffer in memory
    # cv2.IMREAD_COLOR ensures it's loaded as a 3-channel BGR image
    # cv2.IMREAD_UNCHANGED can be used if you need to preserve alpha (for PNGs)
    # but typically color cameras publish JPEG (no alpha) or PNG (can have alpha)
    # For consistency, let's aim for a BGR image. If alpha is needed later, adjust.
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 
    
    if cv_image is None:
        raise ValueError(f"cv2.imdecode failed for format {img_format}. Data length: {len(compressed_data)}")
        
    # cv_image is typically BGR after imdecode
    return cv_image

class DualCameraLegoClient:
    def __init__(self, task, rosbridge_url='ws://localhost:9090', inference_frequency=1.0): # Added inference_frequency
        self.rosbridge_url = rosbridge_url
        self.task = task # assembly_key
        self.inference_frequency = inference_frequency # Hz


        # --- Parameters (adjust as needed, or load from config file/env vars) ---
        self.cam1_topic = '/cam_destroyer/color/image_raw/compressed'
        self.cam2_topic = '/cam_architect/color/image_raw/compressed'
        self.lego_topic = '/lego/assembling_step'
        
        # Paths should be valid within the Docker container
        sim_data_root = 'outputs' 
        sam2_checkpoint = './checkpoints/sam2.1_hiera_large.pt'
        sam2_config = 'configs/sam2.1/sam2.1_hiera_l.yaml'
        device = 'cuda' # or 'cpu'
        
        self.output_cutout1_topic = '/cam_destroyer/lego_cutout'
        self.output_cutout2_topic = '/cam_architect/lego_cutout'
        self.output_detection_topic = '/lego/completed_step' # Publishes std_msgs/String

        # Ensure paths are absolute or correctly relative to the script's location in Docker
        script_dir = Path(__file__).parent.resolve()
        sim_data_root = str(script_dir / sim_data_root)
        #sam2_checkpoint = str(script_dir / sam2_checkpoint)
        #sam2_config = str(script_dir / sam2_config)
        print(f"Using sim_data_root: {sim_data_root}, sam2_checkpoint: {sam2_checkpoint}, sam2_config: {sam2_config}")
        
        self.inferer = OnlineLegoInferer(
            sim_data_root_dir=sim_data_root, # Use absolute path if necessary
            sam2_checkpoint_path=sam2_checkpoint, # Use absolute path
            sam2_model_config_path=sam2_config, # Use absolute path
            device=device
        )
        print("OnlineLegoInferer initialized.")

        self.latest_image_cam1 = None
        self.latest_image_cam2 = None
        self.latest_header_cam1 = None
        self.latest_header_cam2 = None
        self.latest_assembling_step = -1
        self.latest_data_lock = asyncio.Lock() # To protect access to latest_image_camX and latest_header_camX


    async def publish_message(self, websocket, topic_name, msg_type, msg_data):
        publish_msg = {
            "op": "publish",
            "topic": topic_name,
            "msg": msg_data,
            "type": msg_type
        }
        await websocket.send(json.dumps(publish_msg))
        # print(f"Sent message to {topic_name}")

    async def subscribe_to_topic(self, websocket, topic_name, msg_type="sensor_msgs/CompressedImage"):
        subscribe_msg = {
            "op": "subscribe",
            "topic": topic_name,
            "type": msg_type,
            "throttle_rate": 50, # ms, adjust as needed (e.g., 100ms for 10Hz)
            "queue_length": 1    # Get only the latest
        }
        await websocket.send(json.dumps(subscribe_msg))
        print(f"Subscribed to {topic_name}")

    async def image_handler_cam1(self, msg):
        # print("Received image from cam1")
        try:
            async with self.latest_data_lock:
                self.latest_image_cam1 = ros_compressed_image_json_to_cv_image(msg['msg'])
                self.latest_header_cam1 = msg['msg']['header']
            # DO NOT call try_process_images here anymore
        except Exception as e:
            print(f"Error processing cam1 image: {e}")

    async def image_handler_cam2(self, msg):
        # print("Received image from cam2")
        try:
            async with self.latest_data_lock:
                self.latest_image_cam2 = ros_compressed_image_json_to_cv_image(msg['msg'])
                self.latest_header_cam2 = msg['msg']['header']
            # DO NOT call try_process_images here anymore
        except Exception as e:
            print(f"Error processing cam2 image: {e}")

    async def handler_assembly_step(self, msg):
        # print("Received assembly step")
        try:
            async with self.latest_data_lock:
                self.latest_assembling_step = int(msg['msg']['data'])
        except Exception as e:
            print(f"Error processing assembly step: {e}")
            
    async def _perform_inference_and_publish(self, img_cam1_np_orig, img_cam2_np_orig, header_cam1_orig, header_cam2_orig, cur_assembling_step):
        """
        Contains the core logic for inference and publishing results.
        Takes copies of images and headers to process.
        """
        print(f"Performing inference for task: {self.task}")

        # Convert BGR (from ros_compressed_image_json_to_cv_image) to RGB for inferer
        img_cam1_rgb = cv2.cvtColor(img_cam1_np_orig, cv2.COLOR_BGR2RGB)
        img_cam2_rgb = cv2.cvtColor(img_cam2_np_orig, cv2.COLOR_BGR2RGB)

        results = self.inferer.infer_dual_camera(
            img_cam1_rgb,
            img_cam2_rgb,
            self.task, # assembly_key,
            cur_assembling_step=cur_assembling_step,
        )
        best_sim_id = results.best_sim_id
        best_score = results.best_score

        #written by yijie to add execution check
        assembly_complete = best_score > 0.9
        assembly_msg_ros = {"data": assembly_complete}

        # 发布装配完成的bool消息
        await self.publish_message(self.websocket_connection, '/lego/assembly_complete', "std_msgs/Bool", assembly_msg_ros)
        print(f"Published assembly completion: {assembly_complete}")


        visualize_path = self.inferer.temp_base_dir / f"visualize_{self.inferer.count:06d}.png"
        visualize_img = lego_visualize.visualize(self.inferer, img_cam1_rgb, img_cam2_rgb, results, cur_assembling_step, save_path=visualize_path)

        # # Publish cutouts (RGBA)
        # if live_cutout_cam1_rgba is not None:
        #     cutout1_msg_data = cv_image_to_ros_image_json(live_cutout_cam1_rgba, "rgba8", header_cam1_orig['frame_id'], header_cam1_orig['stamp']['secs'] + header_cam1_orig['stamp']['nsecs']*1e-9)
        #     await self.publish_message(self.websocket_connection, self.output_cutout1_topic, "sensor_msgs/CompressedImage", cutout1_msg_data)
        
        # if live_cutout_cam2_rgba is not None:
        #     cutout2_msg_data = cv_image_to_ros_image_json(live_cutout_cam2_rgba, "rgba8", header_cam2_orig['frame_id'], header_cam2_orig['stamp']['secs'] + header_cam2_orig['stamp']['nsecs']*1e-9)
        #     await self.publish_message(self.websocket_connection, self.output_cutout2_topic, "sensor_msgs/CompressedImage", cutout2_msg_data)

        # Publish detection result (std_msgs/String)
        detection_data_str = f'step: {best_sim_id}, score: {best_score:.4f}, task: {self.task}'
        detection_msg_ros = {"data": detection_data_str}
        await self.publish_message(self.websocket_connection, self.output_detection_topic, "std_msgs/String", detection_msg_ros)
        print(f"Published detection: {detection_data_str}")

    async def periodic_inference_scheduler(self):
        """
        Periodically checks for new images and triggers inference.
        """
        if self.inference_frequency <= 0:
            print("Inference frequency is not positive, scheduler will not run.")
            return
            
        sleep_duration = 1.0 / self.inference_frequency
        print(f"Inference scheduler started. Frequency: {self.inference_frequency} Hz (Sleep: {sleep_duration:.2f}s)")

        while True:
            img_cam1_to_process = None
            img_cam2_to_process = None
            header_cam1_to_process = None
            header_cam2_to_process = None
            cur_assembling_step = -1

            async with self.latest_data_lock:
                if self.latest_image_cam1 is not None and self.latest_image_cam2 is not None:
                    # Copy data to process outside the lock
                    img_cam1_to_process = self.latest_image_cam1.copy()
                    img_cam2_to_process = self.latest_image_cam2.copy()
                    header_cam1_to_process = self.latest_header_cam1 # Headers are dicts, shallow copy is fine
                    header_cam2_to_process = self.latest_header_cam2
                    cur_assembling_step = self.latest_assembling_step
                    
                    # Optional: Clear them if you only want to process each image pair once
                    # self.latest_image_cam1 = None 
                    # self.latest_image_cam2 = None
            
            if img_cam1_to_process is not None and img_cam2_to_process is not None:
                try:
                    await self._perform_inference_and_publish(
                        img_cam1_to_process, 
                        img_cam2_to_process,
                        header_cam1_to_process,
                        header_cam2_to_process,
                        cur_assembling_step
                    )
                except Exception as e:
                    print(f"Error during scheduled inference: {e}")
            # else:
                # print("Waiting for images from both cameras...")

            await asyncio.sleep(sleep_duration)

    async def run(self):
        print(f"Attempting to connect to rosbridge at {self.rosbridge_url}...")
        async with websockets.connect(self.rosbridge_url, ping_interval=20, ping_timeout=20, max_size=2000000) as websocket:
            print("Connected to rosbridge.")
            self.websocket_connection = websocket # Store for publishing

            await self.subscribe_to_topic(websocket, self.cam1_topic)
            await self.subscribe_to_topic(websocket, self.cam2_topic)
            await self.subscribe_to_topic(websocket, self.lego_topic, msg_type="std_msgs/Int32")

            # Start the periodic inference scheduler as a background task
            scheduler_task = asyncio.create_task(self.periodic_inference_scheduler())

            async for message_str in websocket:
                try:
                    message = json.loads(message_str)
                    topic = message.get("topic")
                    # print(f"Received message on topic: {topic}") # Can be verbose
                    
                    if topic == self.cam1_topic:
                        await self.image_handler_cam1(message)
                    elif topic == self.cam2_topic:
                        await self.image_handler_cam2(message)
                    elif topic == self.lego_topic:
                        await self.handler_assembly_step(message)
                except json.JSONDecodeError:
                    print(f"Could not decode JSON: {message_str}")
                except Exception as e:
                    print(f"Error processing incoming message: {e}")
            
            scheduler_task.cancel() # Cancel scheduler when websocket connection closes
            try:
                await scheduler_task
            except asyncio.CancelledError:
                print("Inference scheduler cancelled.")

        print("Disconnected from rosbridge.")

    def start_client(self):
        try:
            asyncio.run(self.run())
        except KeyboardInterrupt:
            print("Client stopped by user.")
        finally:
            #if hasattr(self.inferer, 'cleanup_all_temp_dirs'):
                #self.inferer.cleanup_all_temp_dirs()
            print("Cleanup complete.")

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Dual Camera Lego Infer Client")
    parser.add_argument('--task', type=str, default='S', help='Task to perform (e.g., S, cliff, etc.)')
    parser.add_argument('--freq', type=float, default=1.0, help='Inference frequency in Hz (e.g., 1.0 for 1Hz, 0.5 for 0.5Hz)')
    args = parser.parse_args()
    current_task = args.task
    inference_hz = args.freq

    # You might need to pass the host IP if Docker cannot resolve 'localhost' to the host
    # rosbridge_server_url = 'ws://<host_ip_address>:9090' 
    rosbridge_server_url = 'ws://localhost:9090' # if Docker networking allows localhost access to host

    client = DualCameraLegoClient(task=current_task, rosbridge_url=rosbridge_server_url, inference_frequency=inference_hz)
    client.start_client()