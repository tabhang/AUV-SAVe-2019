#!/usr/bin/env python
# license removed for brevity
import roslib
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

cap = cv2.VideoCapture(1)

pub = rospy.Publisher('bin_error', String, queue_size = 10)

def talker():
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lr = np.array([110, 20, 15])                      #0
    ur = np.array([180, 255, 255])                                #10
    mask1 = cv2.inRange(hsv_img, lr, ur) 
    mask = mask1
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
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        error_x = cX - 320
        error_y = 180 - cY
        pub_str = str(error_x) + '/' + str(error_y)
        pub.publish(pub_str)
        x,y,w,h = cv2.boundingRect(cnt)
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.circle(frame,(320,180), 5, (0,0,255), -1)
        cv2.circle(frame,(cX,cY), 5, (0,255,0), -1)
    except ValueError:
        pass 


    # Display the resulting frame
    cv2.imshow('frame',frame)
    cv2.waitKey(3)


if __name__ == '__main__':
 try:
  rospy.init_node('logitech_reader', anonymous=True)
  while(1):
   talker()
 except KeyboardInterrupt:
  cap.release()
  cv2.destroyAllWindows()
