#!/usr/bin/env python
import roslib
import rospy 
import serial
from std_msgs.msg import String

port = '/dev/ttyUSB1'

ser = serial.Serial(port, 9600, timeout = 1)
data = 100

Start_byte = 250
Vx = 0
Vy = 0
Vz = 0
yaw = 0      #pitch
pitch = 0    #roll
roll = 0   #yaw

start = 0

def callback(data):
 global roll
 global pitch
 global yaw
 global Vx
 global Vy
 global Vz
 array = data.data.split('/')
 Vx = int(array[0])
 Vy = int(array[1])
 Vz = int(array[2])
 yaw = int(array[3])
 pitch = int(array[4])
 roll = int(array[5])
 if(Vx<0):
  Vx = 100 - Vx
 if(Vy<0):
  Vy = 100 - Vy
 if(Vz<0):
  Vz = 100 - Vz
 if(roll<0):
  roll = 100 - roll
 if(pitch<0):
  pitch = 100 - pitch
 if(yaw<0):
  yaw = 100 - yaw
 print 'Vx:' + str(Vx) +'Vy:' + str(Vy) +'Vz:' + str(Vz) +'v_roll:' + str(roll) +'v_pitch:' + str(pitch) +'v_yaw:' + str(yaw)


print("Running")

if __name__ == '__main__':
 try:
  pub = rospy.Publisher('depth', String, queue_size = 10)
  rospy.init_node('uart_transmit', anonymous=True)
  rospy.Subscriber("/velocities", String, callback)
  while True:
   checksum = Vx + Vy + Vz + roll + pitch + yaw
   checksum_high = int(checksum/256)
   checksum_low = checksum&0xFF
   ser.write(str(chr(Start_byte)))
   ser.write(str(chr(Vx)))
   ser.write(str(chr(Vy)))
   ser.write(str(chr(Vz)))
   ser.write(str(chr(yaw)))                #yaw
   ser.write(str(chr(roll)))               #roll
   ser.write(str(chr(pitch)))            #pitch
   ser.write(str(chr(checksum_low)))          
   ser.write(str(chr(checksum_high)))
   #print 'hello'
   start_byte_rec = ser.readline()
   if(start_byte_rec.rstrip() == '100'):
    start = 1
   else:
    garbage = (ser.readline().rstrip())
   if(start==1):
    temp = (ser.readline().rstrip())
    pub.publish(str(temp))
    #print temp
    start = 0
   rospy.sleep(0.01)
 except KeyboardInterrupt:
  print("Shutting Down")

