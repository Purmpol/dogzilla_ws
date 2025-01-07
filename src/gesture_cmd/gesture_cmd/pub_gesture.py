#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from std_srvs.srv import SetBool

import math
import time
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import mediapipe as mp 

class handDetector(Node):
  def __init__(self, name, mode=False, maxHands=1, detectorCon=0.5, trackCon=0.5):
    super().__init__(name)
    self.tipIds = [4, 8, 12, 16, 20]
    self.mpHand = mp.solutions.hands
    self.mpDraw = mp.solutions.drawing_utils
    self.hands = self.mpHand.Hands(
      static_image_mode=mode,
      max_num_hands=maxHands,
      min_detection_confidence=detectorCon,
      min_tracking_confidence=trackCon
    )
    
    self.hand_line_color = (0, 255, 0)  # #00ff00
    self.hand_dot_color = (0, 0, 255)   # #ff0000
    self.lmList = []
    
    self.gesture_record = []
    self.filtered_gesture = ''
    
    # Create subscribers
    self.sub_image =  self.create_subscription(CompressedImage, '/image_raw/compressed', self.listener_callback, 10)
    # Create service
    self.srv = self.create_service(SetBool, 'set_robot_cam', self.handle_set_robot_cam)
       
    # create publishers
    self.pub_gesture = self.create_publisher(String, '/dogzilla/pub_gesture/gesture', 2)
    self.pub_image = self.create_publisher(CompressedImage, '/dogzilla/image_gesture/compressed', 1)
    
    # Declare a parameter with a default value
    def bgr2hex(b,g,r):
      return f'{r<<16|g<<8|b:06X}'
    self.declare_parameter('gesture_colors', [bgr2hex(*self.hand_line_color), bgr2hex(*self.hand_dot_color)])
    
  def handle_set_robot_cam(self, request, response):
    print(f"'set_robot_cam' service called for switching sub image topic: {request.data} ...")
    if not request.data:
      if self.sub_image.topic_name != '/foxglove/compressed':
        self.destroy_subscription(self.sub_image)
        self.sub_image =  self.create_subscription(CompressedImage, '/foxglove/compressed', self.listener_callback, 10)
        print("switch to foxglove camera")
    else:
      if self.sub_image.topic_name != '/image_raw/compressed':
        self.destroy_subscription(self.sub_image)
        self.sub_image =  self.create_subscription(CompressedImage, '/image_raw/compressed', self.listener_callback, 10)
        print("switch to robot camera")
    response.success = True
    return response
    
  def get_dist(self, point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))

  def calc_angle(self, pt1, pt2, pt3):
    point1 = self.lmList[pt1][1], self.lmList[pt1][2]
    point2 = self.lmList[pt2][1], self.lmList[pt2][2]
    point3 = self.lmList[pt3][1], self.lmList[pt3][2]
    a = self.get_dist(point1, point2)
    b = self.get_dist(point2, point3)
    c = self.get_dist(point1, point3)
    try:
      radian = math.acos((math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) / (2 * a * b))
      angle = radian / math.pi * 180
    except:
      angle = 0
    return abs(angle)


  def findHands(self, frame, hand_line_color, hand_dot_color, draw=True  ):
    self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=hand_dot_color, thickness=-1, circle_radius=6)
    self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=hand_line_color, thickness=2, circle_radius=2)
    
    self.lmList = []
    img = np.zeros(frame.shape, np.uint8)
    img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
    self.results = self.hands.process(img_RGB)
    if self.results.multi_hand_landmarks:
      for i in range(len(self.results.multi_hand_landmarks)):
        if draw: self.mpDraw.draw_landmarks(frame, self.results.multi_hand_landmarks[i], self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
        self.mpDraw.draw_landmarks(img, self.results.multi_hand_landmarks[i], self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
        for id, lm in enumerate(self.results.multi_hand_landmarks[i].landmark):
          h, w, c = frame.shape
          cx, cy = int(lm.x * w), int(lm.y * h)
          self.lmList.append([id, cx, cy])
    return frame, img

  def frame_combine(slef,frame, src):
    if len(frame.shape) == 3:
      frameH, frameW = frame.shape[:2]
      srcH, srcW = src.shape[:2]
      dst = np.zeros((max(frameH, srcH), frameW + srcW, 3), np.uint8)
      dst[:, :frameW] = frame[:, :]
      dst[:, frameW:] = src[:, :]
    else:
      src = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
      frameH, frameW = frame.shape[:2]
      imgH, imgW = src.shape[:2]
      dst = np.zeros((frameH, frameW + imgW), np.uint8)
      dst[:, :frameW] = frame[:, :]
      dst[:, frameW:] = src[:, :]
    return dst

  def fingersUp(self):
    fingers=[]
    # Thumb
    if (self.calc_angle(self.tipIds[0],
                        self.tipIds[0] - 1,
                        self.tipIds[0] - 2) > 150.0) and (
            self.calc_angle(
                self.tipIds[0] - 1,
                self.tipIds[0] - 2,
                self.tipIds[0] - 3) > 150.0): fingers.append(1)
    else:
      fingers.append(0)
    # 4 finger
    for id in range(1, 5):
      if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
        fingers.append(1)
      else:
        fingers.append(0)
    return fingers

  def get_gesture(self):
    gesture = ""
    fingers = self.fingersUp()
    if self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[1]][2] and \
            self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[2]][2] and \
            self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[3]][2] and \
            self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[4]][2] : gesture = "Thumb_down"

    elif self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[1]][2] and \
            self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[2]][2] and \
            self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[3]][2] and \
            self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[4]][2] and \
            self.calc_angle(self.tipIds[1] - 1, self.tipIds[1] - 2, self.tipIds[1] - 3) < 150.0 : gesture = "Thumb_up"
    if fingers.count(1) == 3 or fingers.count(1) == 4:
        if fingers[0] == 1 and (
                self.get_dist(self.lmList[4][1:], self.lmList[8][1:])<self.get_dist(self.lmList[4][1:], self.lmList[5][1:])
        ): gesture = "OK"
        elif fingers[2] == fingers[3] == 0: gesture = "Rock"
        elif fingers.count(1) == 3: gesture = "Three"
        else: gesture = "Four"
    elif fingers.count(1) == 0: gesture = "Zero"
    elif fingers.count(1) == 1: gesture = "One"
    elif fingers.count(1) == 2:
        if fingers[0] == 1 and fingers[4] == 1: gesture = "Six"
        elif fingers[0] == 1 and self.calc_angle(4, 5, 8) > 90: gesture = "Eight"
        elif fingers[0] == fingers[1] == 1 and self.get_dist(self.lmList[4][1:], self.lmList[8][1:]) < 50: gesture = "Heart_single"
        else: gesture = "Two"
    elif fingers.count(1)==5:gesture = "Five"
    # if self.get_dist(self.lmList[4][1:], self.lmList[8][1:]) < 60 and \
    #         self.get_dist(self.lmList[4][1:], self.lmList[12][1:]) < 60 and \
    #         self.get_dist(self.lmList[4][1:], self.lmList[16][1:]) < 60 and \
    #         self.get_dist(self.lmList[4][1:], self.lmList[20][1:]) < 60 : gesture = "Seven"
    if self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[1]][2] and \
            self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[2]][2] and \
            self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[3]][2] and \
            self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[4]][2] and \
            self.calc_angle(self.tipIds[1] - 1, self.tipIds[1] - 2, self.tipIds[1] - 3) > 150.0 : gesture = "Eight"
    return gesture
  
  def hex_to_bgr(self, hex_string):
      r_hex = hex_string[1:3]
      g_hex = hex_string[3:5]
      b_hex = hex_string[5:7]
      return int(b_hex, 16), int(g_hex, 16), int(r_hex, 16)
    
  
  def listener_callback(self, msg):
    self.hand_line_color = self.hex_to_bgr(self.get_parameter('gesture_colors').value[0])
    self.hand_dot_color = self.hex_to_bgr(self.get_parameter('gesture_colors').value[1])
       
    np_arr = np.frombuffer(msg.data, np.uint8)
    frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)
    frame, img = self.findHands(frame, draw=True, 
                                hand_line_color=self.hand_line_color, 
                                hand_dot_color=self.hand_dot_color) # draw in the same image
    ### Gesture Text & Publish ###
    if len(self.lmList) != 0:  
      totalFingers = str(self.get_gesture())
      
      self.gesture_record.append(totalFingers)
      if len(self.gesture_record) > 10:
        self.filtered_gesture = numpy_mode(self.gesture_record)
        # print(self.filtered_gesture)
        self.gesture_record.clear()         
      
      cv.rectangle(frame, (0, 430), (150, 480), (0, 255, 0), cv.FILLED)
      cv.putText(frame, self.filtered_gesture, (10, 470), cv.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
      msg = String()
      msg.data = str(self.filtered_gesture)
      self.pub_gesture.publish(msg)
        
    
    #### Create CompressedIamge ####
    msg_img = CompressedImage()
    msg_img.header.stamp =  self.get_clock().now().to_msg()
    msg_img.format = "jpeg"
    msg_img.data = cv.imencode('.jpg', frame)[1].tobytes()   
    # Publish new image    
    self.pub_image.publish(msg_img)
    
    # cv.imshow('frame', frame)
    # cv.waitKey(1)

  
def numpy_mode(data):
  
    # Index and counts of all elements in the array
    (sorted_data, idx, counts) = np.unique(data, return_index=True, return_counts=True)

    # Index of element with highest count (i.e. the mode)
    index = idx[np.argmax(counts)]

    # Return the element with the highest count
    return data[index]

'''
Zero One Two Three Four Five Six Seven Eight
Ok: OK
Rock: rock
Thumb_up : 点赞
Thumb_down: 拇指向下
Heart_single: 单手比心
'''

def main():
  print("start it")
  rclpy.init() 
  
  hand_detector = handDetector('hand_gesture', detectorCon=0.75)
  rclpy.spin(hand_detector)
  hand_detector.destroy_node()
  rclpy.shutdown()

