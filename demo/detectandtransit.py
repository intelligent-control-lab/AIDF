import rospy
from geometry_msgs.msg import Point
from yk_msgs.srv import SetPose, SetPoseRequest
import time
import pdb
import argparse

ROBOT_NAMESPACE = "yk_architect"

# Callback function to handle the received message
def box_center_callback(msg):
    print("Received box center coordinates:", msg)

    # Extract x, y, z from the Point message
    x = msg.x
    y = msg.y
    z = msg.z

    # Adjust z to be 3 inches (0.0762 meters) above the target location
    x += 0.2
    z += 0.4
    # pdb.set_trace()

    # time.sleep(1)  # Wait for 2 seconds before moving
    # Command the robot to move to the target location
    send_robot_request(x, y, z)

def send_robot_request(x, y, z, ori_w=0, ori_x=1, ori_y=0, ori_z=0):
    rospy.wait_for_service(f'/{ROBOT_NAMESPACE}/yk_set_pose')
    set_pose = rospy.ServiceProxy(f'/{ROBOT_NAMESPACE}/yk_set_pose', SetPose)

    request = SetPoseRequest()
    request.pose.position.x = x
    request.pose.position.y = y
    request.pose.position.z = z
    request.pose.orientation.w = ori_w  # Assuming no rotation, set w to 0
    request.pose.orientation.x = ori_x
    request.pose.orientation.y = ori_y
    request.pose.orientation.z = ori_z
    print("Setting pose to:", request.pose)

    try:
        response = set_pose(request)
        print("Robot moved to target location:", response.pose)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

# Add argument parsing for mode selection
parser = argparse.ArgumentParser(description="Detect and Transit Script")
parser.add_argument('--mode', choices=['one-time', 'loop'], default='one-time', help="Mode of operation: 'one-time' or 'loop'")
args = parser.parse_args()

if __name__ == "__main__":
    # rospy.init_node("detect_and_transit")

    # Subscribe to the topic with the correct message type
    # rospy.Subscriber("/yk_architect/dinov2_box_center", Point, box_center_callback)

    print("Subscribed to /yk_architect/dinov2_box_center")

    if args.mode == "loop":
        print("Running in loop mode")
        start_poiny = (0.2, 0, 0.4)
        for i in range(10):
            x = start_poiny[0] + i * 0.01
            y = start_poiny[1]
            z = start_poiny[2]
            send_robot_request(x, y, z)
        # while True:
        #     rospy.sleep(2)  # Wait 2 second before next detection
    else:
        print("Running in one-time mode")
        # rospy.sleep(1)  # Allow time for the callback to process a message
        # rospy.signal_shutdown("One-time mode completed")
        send_robot_request(0.2, 0, 0.4)