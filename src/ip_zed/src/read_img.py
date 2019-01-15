#!/usr/bin/env python
import roslib
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()


def callback(data):
  cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
  gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
  blur = cv2.GaussianBlur(gray,(5,5),0)
  ret3,th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
  # erosion and dilation
  kernel = np.ones((5,5),np.uint8)
  erosion = cv2.erode(th3,kernel,iterations = 2) 
  dilation = cv2.dilate(erosion,kernel,iterations = 2)
  # canny edge
  upper = ret3
  lower = upper*0.5
  edges = cv2.Canny(blur, 100, 200)
  # contours
  _, contours,hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  areas = [cv2.contourArea(c) for c in contours]
  max_index = np.argmax(areas)
  cnt=contours[max_index]
  x,y,w,h = cv2.boundingRect(cnt)
  cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
  # result
  cv2.imshow("Result", cv_image)
  cv2.waitKey(3)


def listner():
 rospy.init_node('zed_image_reader', anonymous=True)
 rospy.Subscriber("/zed/right/image_raw_color", Image, callback)
 rospy.spin()


if __name__ == '__main__':
 try:
  listner()
 except KeyboardInterrupt:
  print("Shutting Down")
 cv2.destroyAllWindows()

 
