#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
import math

pub = rospy.Publisher('velocities', String, queue_size = 10)

required_depth = 980
required_yaw = 163
required_pitch = 0
required_roll = 0
offset_x = 0
offset_z = 0
error_depth = 0
depth = 0
disable_correction=0
ip_data_counter = 0
v_yaw = 0
v_pitch = 0
v_roll = 0
distance_counter = 0

#cmd velocities
Vx = 0
Vy = 0
Vz = 0
yaw = 0
pitch = 0
roll = 0

# angle kps
Kp_yaw = 15          #7
Kp_pitch = 7
Kp_roll = 2
Kp_depth = 4

# camera kps
Kp_camera_z = 0.01
Kp_camera_x = 0.01
Kp_camera_yaw = 0.3

#weights
Weight_camera_x = 0
Weight_camera_z = 0
Weight_camera_yaw = 0
Weight_depth = 1
Weight_yaw = 1
Weight_pitch = 1
Weight_roll = 0
Weight_imu = 0

#co-ordinates
x_curr = 0
y_curr = 0
z_curr = 0

object_detected=0


def callback(data):
 global depth
 depth_string = data.data.rstrip().split('.')
 depth = int(depth_string[0])              #ye niche lena hai
 #print depth 


def getimu(data):
 global yaw
 global pitch
 global roll
 quatx = data.orientation.x
 quaty = data.orientation.y
 quatz = data.orientation.z
 quatw = data.orientation.w
 quaternion = (quatx, quaty, quatz, quatw)
 euler = tf.transformations.euler_from_quaternion(quaternion)
 roll = euler[0]*(180/3.14)
 pitch = euler[1]*(180/3.14)
 yaw = euler[2]*(180/3.14)
 print str(roll) + ' / ' + str(pitch) + ' / ' + str(yaw) 


def ipdata(data):
 global Vy
 global offset_z
 global offset_x
 global object_detected
 global ip_data_counter
 object_detected = 1
 temp = data.data
 array = temp.split('/')
 offset_z = int(array[0])
 offset_x = int(array[1])*(-1)
 ip_data_counter=0 


def zdata(odom_data):
 global x_curr
 global y_curr
 global z_curr
 x_curr = odom_data.pose.pose.position.x
 y_curr = odom_data.pose.pose.position.y
 z_curr = odom_data.pose.pose.position.z
 #print str(x_curr) + ' / ' + str(y_curr) + ' / ' + str(z_curr)


def listner():
 rospy.init_node('correction', anonymous=True)
 rospy.Subscriber("/depth", String, callback)
 rospy.Subscriber("/vectornav/IMU", Imu, getimu)
 rospy.Subscriber("/red_error", String, ipdata)
 rospy.Subscriber("/zed/odom", Odometry, zdata)
 

def process():
 global Vx
 global Vy
 global Vz
 global error_depth
 global disable_correction
 global object_detected
 global ip_data_counter
 global v_yaw
 global v_pitch
 global v_roll
 global distance_counter
 global Weight_yaw
 distance_counter = distance_counter+1
# ......................................................................error calculations
 error_depth = required_depth - depth
 error_yaw = (1)*(required_yaw - yaw)         #in timer maybe?
 error_pitch = (required_pitch - int(pitch))
 error_roll = required_roll - roll
#........................................................................navigation cases
 disable_correction=1
 max_disable = 0
 if(distance_counter>1000 and distance_counter<3272):             #1136 -> 2m
  Vy = 30
 elif(distance_counter>3272 and distance_counter<3500):
  Vy = 0
  v_yaw = 70
  max_disable = 1
 elif(distance_counter>3500):
  max_disable = 1
  if(yaw<90 and yaw>0):                  #scan for red object in 180deg
   v_yaw = 70
  if(yaw>-90 and yaw<0):
   v_yaw = -70
  
 #print distance_counter
#......................................................................error corrections
 Vz = int(offset_z*Kp_camera_z*Weight_camera_z + error_depth*Kp_depth*Weight_depth)
 v_pitch = int(error_pitch*Kp_pitch*Weight_pitch)
 if(max_disable == 0):
  v_yaw = int(error_yaw*Kp_yaw*Weight_yaw + offset_x*Kp_camera_yaw*Weight_camera_yaw)
 if(disable_correction==0):
  Vx = int(offset_x*Kp_camera_x*Weight_camera_x)
  v_roll = int(error_roll*Kp_roll*Weight_roll)
#........................................................................................... #
 if(Vx>100):
  Vx = 100
 if(Vx<-100):
  Vx = -100
 if(Vy>100):
  Vy = 100
 if(Vy<-100):
  Vy = -100
 if(Vz>100):
  Vz = 100
 if(Vz<-100):
  Vz = -100
 if(v_yaw>100):
  v_yaw = 100
 if(v_yaw<-100):
  v_yaw = -100
 if(v_pitch>100):
  v_pitch = 100
 if(v_pitch<-100):
  v_pitch = -100
 if(v_roll>100):
  v_roll = 100
 if(v_roll<-100):
  v_roll = -100
 pub_string = str(Vx) + '/' + str(Vy) + '/' + str(Vz) + '/' + str(v_yaw) + '/' + str(v_pitch) + '/' + str(v_roll)
 #pub_string
 pub.publish(pub_string)
 rospy.sleep(0.01)


if __name__ == '__main__':
 try:
  listner()
  while True:
   process()
 except KeyboardInterrupt:
  print("Shutting Down")

