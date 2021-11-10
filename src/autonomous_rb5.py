
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import json
import sys
import os
import time
import numpy as np
import cv2
import onnxruntime

class AutonomousRB5(Node):
    def __init__(self):
        super().__init__('autonomous_rb5')

        self.linear_velocity = [0.0, 0.15, -0.1, 0.1]  # unit: m/s
        self.lin_decision = ['No Movement', 'Forward', 'Reverse', 'Slow Speed']
        self.angular_velocity = [0.0, -0.7, 0.7]  # unit: m/s
        self.ang_decision = ['No Turn', 'Turning Right', 'Turning Left']
        self.obstacle = [True, True, True, True] # Values for Front, Back, Right, Left. True if obstacle present False if obstacle is not available.
        self.safety_distance = 0.3  # unit: m
        self.scan_ranges = []
        self.init_scan_state = False  # To get the initial scan data at the beginning

        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_vel_raw_callback, qos)
        self.update_timer = self.create_timer(0.010, self.update_callback)
        self.get_logger().info("Autonomous Drive - RB5 is Initialized!.")

        # Model
        self.casecade = cv2.CascadeClassifier("model/stop_sign_classifier_2.xml")
        model="model/model.onnx"

        self.video = cv2.VideoCapture(0)
        frame_width = int(self.video.get(3))
        frame_height = int(self.video.get(4))
        self.out = cv2.VideoWriter('rb5_auto_drive_output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (frame_width,frame_height))
        self.session = onnxruntime.InferenceSession(model, None)

        self.input_name1 = self.session.get_inputs()[0].name
        self.input_name2 = self.session.get_inputs()[1].name
        self.output_name = self.session.get_outputs()[0].name

        self.action_masks = np.array([[1, 1, 1, 1, 1, 1]]).astype("float32") #list of possible actions that model can predict

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.front = self.scan_ranges[315:359] + self.scan_ranges[0:45]
        while 0.0 in self.front:
            self.front.remove(0.0)
        self.right = self.scan_ranges[45:135]
        while 0.0 in self.right:
            self.right.remove(0.0)
        self.back = self.scan_ranges[135:225]
        while 0.0 in self.back:
            self.back.remove(0.0)
            #print(self.back)
        self.left = self.scan_ranges[225:315]
        while 0.0 in self.left:
            self.left.remove(0.0)

        self.obstacle = [(min(self.front) < self.safety_distance),
                (min(self.back) < self.safety_distance),
                (min(self.right) < self.safety_distance),
                (min(self.left) < self.safety_distance)]

        self.init_scan_state = True


    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_callback(self):
        if self.init_scan_state:
            self.run_modules()

    def run_modules(self):
        twist = Twist()
        
        #twist.linear.x = 0.0
        #twist.angular.z = 0.0

        #self.cmd_vel_pub.publish(twist)

        ret, frame = self.video.read()
        #img = cv2.imread("path.jpg")
        imgs = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        stop = self.casecade.detectMultiScale(imgs, 1.2, 5)

        if (len(stop) > 0):
            for (x, y, w, h) in stop:
                frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (0,0,255), 2)

            txt = "Stop Sign Detected!"
            indx_lin = 0
            indx_ang = 0
        elif(self.obstacle[0]):
            txt = "Obstacle Detected!"
            indx_lin = 0
            indx_ang = 0
        else:
            img = cv2.resize(frame ,(84, 84))
            img.resize(1, 3, 84, 84)
            img = np.array(img).astype("float32")/255.0
            result = self.session.run([self.output_name], {self.input_name1: img,
                self.input_name2:self.action_masks}) #inference
            res = np.exp(result)[0][0]
            lin = res[:3]
            ang = res[3:]
            indx_lin = np.argmax(lin)
            indx_ang = np.argmax(ang)
            txt = "Auto Driving!"

            if indx_ang != 0:
                indx_lin = 3
            #print(indx_lin, indx_ang)
        txt += ' - '+self.lin_decision[indx_lin]+', '+self.ang_decision[indx_ang]+'.'
        frame = cv2.putText(frame, txt, (20, 20), 1, 1.3, (0,0,200), 2)
        self.out.write(frame)
        self.get_logger().info(txt)
        txt = ''
        twist.linear.x = self.linear_velocity[indx_lin]
        twist.angular.z = self.angular_velocity[indx_ang]
        
        self.cmd_vel_pub.publish(twist)
