#!/usr/bin/env python
import roslib
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()


Cy = 320
Cx = 180

pub = rospy.Publisher('proc_depth', String, queue_size = 10)


def callback_depth(data):
 cv_image = bridge.imgmsg_to_cv2(data, "32FC1")
 depth_image = np.array(cv_image, dtype=np.float32)
 proc_depth = depth_image[Cy,Cx]
 pub.publish(str(proc_depth))
 #print str(Cy) + '/' + str(Cx)
 #print depth_image.shape


def callback_cords(data):
 global Cy
 global Cx
 temp = data.data
 array = temp.split('/')
 Cy = int(array[0])
 Cx = int(array[1])
 #print str(Cy) + '/' + str(Cx)
 

def callback_img(data):
 img = bridge.imgmsg_to_cv2(data, "passthrough")
 cv2.circle(img,(Cx,Cy), 5, (0,255,0), -1)
 cv2.imshow('res',img)
 cv2.waitKey(3)


def listner():
 rospy.init_node('zed_read_depth', anonymous=False)
 rospy.Subscriber("/zed/depth/depth_registered", Image, callback_depth)
 rospy.Subscriber("/zed/right/image_raw_color", Image, callback_img)
 rospy.Subscriber("/depth_cords", String, callback_cords)
 rospy.spin()


if __name__ == '__main__':
 try:
  listner()
 except KeyboardInterrupt:
  print("Shutting Down")
 cv2.destroyAllWindows()

