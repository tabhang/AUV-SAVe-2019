#!/usr/bin/env python
import roslib
import rospy 
import serial
from std_msgs.msg import String

port = '/dev/ttyUSB1'

ser = serial.Serial(port, 9600, timeout = 10)
data = 10

Start_byte = 250
Vx = 0
Vy = 0
Vz = 0
yaw = 0
pitch = 0
roll = 0
 

print("Zero")

if __name__ == '__main__':
 try:
  rospy.init_node('uart_zero', anonymous=True)
  #rospy.Subscriber("/velocities", String, callback)
  while(1):
   checksum = Vx + Vy + Vz + roll + pitch + yaw
   checksum_high = int(checksum/256)
   checksum_low = checksum&0xFF
   ser.write(str(chr(Start_byte)))
   ser.write(str(chr(Vx)))
   ser.write(str(chr(Vy)))
   ser.write(str(chr(Vz)))
   ser.write(str(chr(roll)))
   ser.write(str(chr(pitch)))
   ser.write(str(chr(yaw)))
   ser.write(str(chr(checksum_low)))
   ser.write(str(chr(checksum_high)))
   print 'Vx:' + str(Vx) +'Vy:' + str(Vy) +'Vz:' + str(Vz) +'v_roll:' + str(roll) +'v_pitch:' + str(pitch) +'v_yaw:' + str(yaw)  
 except KeyboardInterrupt:
  print("Shutting Down")

