#!/usr/bin/env python
import roslib
import rospy 
import serial
from std_msgs.msg import String

port = '/dev/ttyUSB2'

ser = serial.Serial(port, 9600, timeout = 5)

start = 0

print("Running")

if __name__ == '__main__':
 try:
  pub = rospy.Publisher('depth', String, queue_size = 10)
  rospy.init_node('read_depth', anonymous=True)
  while(1):
   start_byte = ser.readline()
   if(start_byte.rstrip() == '100'):
    start = 1
   if(start==1):
    temp = (ser.readline().rstrip())
    check = (ser.readline().rstrip())
    pub.publish(str(temp))
    print temp
    start = 0
 except KeyboardInterrupt:
  print("Shutting Down")

