#!/usr/bin/env python
import roslib
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()

error_z = 0
error_y = 0
pub = rospy.Publisher('heart_error', String, queue_size = 10)


def callback(data):
 global error_z
 global error_y
 cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
 #height, width = cv_image.shape[:2]
 #print str(height) + '...' + str(width)
 hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
 lr = np.array([0, 0, 50])                      #0
 ur = np.array([180, 190, 255])                                #10
 mask1 = cv2.inRange(hsv_img, lr, ur)
 lr = np.array([90, 50, 50])
 ur = np.array([100, 255, 255])
 mask2 = cv2.inRange(hsv_img, lr, ur) 
 mask = mask1 #+ mask2
 mask_send = bridge.cv2_to_imgmsg(mask, "passthrough")
 #output = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)
 kernel = np.ones((5,5),np.uint8)                      #erosion dilation
 erosion = cv2.erode(mask,kernel,iterations = 3)
 dilation = cv2.dilate(erosion,kernel,iterations = 3)
 _, contours,hierarchy = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
 areas = [cv2.contourArea(c) for c in contours]
 try:
  max_index = np.argmax(areas)
  cnt=contours[max_index]
  M = cv2.moments(cnt)
  max_area = cv2.contourArea(cnt)
  print max_area
  cX = int(M["m10"] / M["m00"])
  cY = int(M["m01"] / M["m00"])
  error_y = cX - 320
  error_z = 180 - cY
  pub_str = str(error_z) + '/' + str(error_y) + '/' + str(max_area)
  pub.publish(pub_str)
  x,y,w,h = cv2.boundingRect(cnt)
  cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
  cv2.circle(cv_image,(320,180), 5, (0,0,255), -1)
  cv2.circle(cv_image,(cX,cY), 5, (0,255,0), -1)
 except ValueError:
  pass 
 cv2.imshow("Result", cv_image)
 cv2.waitKey(3)


def listner():
 rospy.init_node('zed_red_processor', anonymous=True)
 rospy.Subscriber("/zed/right/image_raw_color", Image, callback)
 rospy.spin()


if __name__ == '__main__':
 try:
  listner()
 except KeyboardInterrupt:
  print("Shutting Down")
 cv2.destroyAllWindows()
