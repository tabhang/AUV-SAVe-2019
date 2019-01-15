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
 hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
 lr = np.array([0, 50, 50])
 ur = np.array([10, 255, 255])
 mask1 = cv2.inRange(hsv_img, lr, ur)
 lr = np.array([170, 50, 50])
 ur = np.array([180, 255, 255])
 mask2 = cv2.inRange(hsv_img, lr, ur) 
 mask = mask1+mask2
 output = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
 cv2.imshow("Result", output)
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
