#!/usr/bin/env python
import roslib
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()

def nothing(x):
    pass


cv_image = 0

def callback(data):
  global cv_image
  cv_image = bridge.imgmsg_to_cv2(data, "passthrough")


def listner():
 rospy.init_node('zed_image_reader', anonymous=True)
 rospy.Subscriber("/zed/right/image_raw_color", Image, callback)


if __name__ == '__main__':
 try:
  listner()
  #Creating a window for later use
  cv2.namedWindow('result')
  # Starting with 100's to prevent error while masking
  h1,s1,v1 = 100,100,100
  h2,s2,v2 = 100,100,100

  # Creating track bar
  cv2.createTrackbar('h1', 'result',0,179,nothing)
  cv2.createTrackbar('s1', 'result',0,255,nothing)
  cv2.createTrackbar('v1', 'result',0,255,nothing)
  cv2.createTrackbar('h2', 'result',0,179,nothing)
  cv2.createTrackbar('s2', 'result',0,255,nothing)
  cv2.createTrackbar('v2', 'result',0,255,nothing)
  while True:
   # get info from track bar and appy to result
   h_l = cv2.getTrackbarPos('h1','result')
   s_l = cv2.getTrackbarPos('s1','result')
   v_l = cv2.getTrackbarPos('v1','result')
   h_h = cv2.getTrackbarPos('h2','result')
   s_h = cv2.getTrackbarPos('s2','result') 
   v_h = cv2.getTrackbarPos('v2','result')
   
   # Normal masking algorithm
   lower_blue = np.array([h_l,s_l,v_l])
   upper_blue = np.array([h_h,s_h,v_h])
   #converting to HSV
   hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

   mask = cv2.inRange(hsv,lower_blue, upper_blue)

   result = cv2.bitwise_and(cv_image,cv_image,mask = mask)
 
   cv2.imshow('result',result)
   cv2.waitKey(3)
   rospy.sleep(0.01)
 except KeyboardInterrupt:
  print("Shutting Down")
 cv2.destroyAllWindows()

 
