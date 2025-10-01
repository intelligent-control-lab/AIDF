#!/usr/bin/env python3
import rospy

import numpy as np
import torch
import torchvision.transforms as T
from PIL import Image
import pickle
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

import os
import sys

class PickPlaceClassifyNode:
    def __init__(self, robot_name):
        self.robot_name = robot_name

        # load model
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../models/pick_place_svm_061325.pkl')
        with open(model_path, 'rb') as file:
            self.clf = pickle.load(file)
        self.model = torch.hub.load("facebookresearch/dinov2", "dinov2_vits14")
        self.device = torch.device('cuda' if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)
        self.bridge = CvBridge()

        self.transformation = T.Compose([T.Resize(224), 
                                            T.CenterCrop(224), 
                                            T.Grayscale(num_output_channels=3),
                                            T.ToTensor(), 
                                            T.Normalize([0.5], [0.5]),
                                            ])

        self.result_pub = rospy.Publisher(f'/{robot_name}/pick_place_classify', Bool, queue_size=1)
        rospy.Subscriber(f'/{robot_name}/gen3_image/compressed', CompressedImage, self.image_callback)
        
        rospy.loginfo("PickPlaceClassifyNode initialized. Waiting for images...")

    def pick_place_prediction(self, cv2_img):
        pil_img = Image.fromarray(cv2_img)
        transformed_img = self.transformation(pil_img)[:3].unsqueeze(0)

        with torch.no_grad():
            embedding = self.model(transformed_img.to(self.device))
            prediction = self.clf.predict(np.array(embedding[0].cpu()).reshape(1, -1))
        
        if prediction == 'success':
            return 1  # in hand
        elif prediction == 'failure':
            return 0

    def image_callback(self, msg):
        rospy.loginfo("Received image")
        result = Bool()

        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='rgb8')
        result.data = self.pick_place_prediction(img)

        self.result_pub.publish(result)
        rospy.loginfo(f"{robot_name} result: %s", result.data)

if __name__ == '__main__':
    robot_name = sys.argv[1] 
    assert(robot_name in ['yk_architect', 'yk_builder', 'yk_creator', 'yk_destroyer'])

    rospy.init_node(f'{robot_name}_pick_place_classify_node', anonymous=False)
    node = PickPlaceClassifyNode(robot_name)
    rospy.spin()