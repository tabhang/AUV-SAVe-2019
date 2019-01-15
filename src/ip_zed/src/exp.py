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
 cv2.imshow('result',cv_image)
 cv2.waitKey(3)


def listner():
 rospy.init_node('zed_read', anonymous=False)
 rospy.Subscriber("/zed/rgb/image_rect_color", Image, callback)
 rospy.spin()


if __name__ == '__main__':
 try:
  listner()
 except KeyboardInterrupt:
  print("Shutting Down")
 cv2.destroyAllWindows()

