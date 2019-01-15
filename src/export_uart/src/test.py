#!/usr/bin/env python
import roslib
import rospy 
import serial
from std_msgs.msg import String

port = '/dev/ttyUSB1'

ser = serial.Serial(port, 9600, timeout = 5)
data = 10
Vz = 0
 

print("Running")

if __name__ == '__main__':
 try:
  while(1):
   ser.write(str(chr(data)))  
 except KeyboardInterrupt:
  print("Shutting Down")

