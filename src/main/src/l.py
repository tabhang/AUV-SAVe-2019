#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
import math

pub = rospy.Publisher('velocities', String, queue_size = 10)
pubc = rospy.Publisher('depth_cords', String, queue_size = 10)

required_depth = 980
required_yaw = 145
required_pitch = 0
required_roll = 0
offset_x = 0
offset_z = 0
offset_y = 0
error_depth = 0
depth = 0
disable_correction=0
ip_data_counter = 0
v_yaw = 0
v_pitch = 0
v_roll = 0
distance_counter = 0
moving_towards_object = 0
error_yaw = 0
zed_depth = 0

#navigation
navi_choice = 0

#cmd velocities
Vx = 0
Vy = 0
Vz = 0
yaw = 0
pitch = 0
roll = 0

# angle kps
Kp_yaw = 7 #15          #7
Kp_pitch = 5   #7
Kp_roll = 2
Kp_depth = 4

# camera kps
Kp_camera_z = 0.7
Kp_camera_x = 0.01
Kp_camera_y = 0.01
Kp_camera_yaw = 0.7

#weights
Weight_camera_x = 0
Weight_camera_y = 0
Weight_camera_z = 0
Weight_camera_yaw = 0
Weight_depth = 1
Weight_yaw = 1
Weight_pitch = 1
Weight_roll = 0
Weight_imu = 0

#correction flags
disable_Vx = 0
disable_Vy = 0
disable_Vz = 0
disable_roll = 0
disable_pitch = 0
disable_yaw = 0
disable_red = 0
disable_white = 1
disable_bin = 1
stop_distance_counter = 0

#co-ordinates
x_curr = 0
y_curr = 0
z_curr = 0

object_detected=0


def callback(data):  #pressure sensor ka
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
 yaw_temp = euler[2]*(180/3.14)
 #if(roll_temp<0):
  #roll = 180 - roll_temp
 #else:
  #roll = roll_temp
 #if(pitch_temp<0):
  #pitch = 180 - pitch_temp
 #else:
  #pitch = pitch_temp
 if(yaw_temp<0):
  yaw = 180 - yaw_temp
 else:
  yaw = yaw_temp
 #print str(roll) + ' / ' + str(pitch) + ' / ' + str(yaw) 


def ipdata(data):
 global Vy
 global offset_z
 global offset_x
 global object_detected
 global ip_data_counter
 if(disable_red == 0):
  object_detected = 1
  temp = data.data
  array = temp.split('/')
  offset_z = int(array[0])*(-1) - 100
  offset_x = int(array[1])*(1)
  Cy = 180 - int(array[0])
  Cx = int(array[1]) + 320
  pub_str = str(Cy) + '/' + str(Cx)
  pubc.publish(pub_str)
  ip_data_counter=0


def ipdata_white(data):
 global Vy
 global offset_z
 global offset_x
 global object_detected
 global ip_data_counter
 if(disable_white == 0):
  object_detected = 1
  temp = data.data
  array = temp.split('/')
  offset_z = int(array[0])*(-1)
  offset_x = int(array[1])*(1)
  Cy = 180 - int(array[0])
  Cx = int(array[1]) + 320
  pub_str = str(Cy) + '/' + str(Cx)
  pubc.publish(pub_str)
  ip_data_counter=0


def ipdata_bin(data):
 global Vy
 global offset_y
 global offset_x
 global object_detected
 global ip_data_counter
 if(disable_white == 0):
  object_detected = 1
  temp = data.data
  array = temp.split('/')
  offset_y = int(array[0])*(-1) - 100
  offset_x = int(array[1])*(1)
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
 rospy.Subscriber("/white_error", String, ipdata_white)
 rospy.Subscriber("/bin_error", String, ipdata_bin)
 rospy.Subscriber("/zed/odom", Odometry, zdata)
 rospy.Subscriber("/proc_depth", String, depth_data)


def depth_data(data):
 global zed_depth
 #if(data.data>-10 and data.data<10):
 zed_depth = float(data.data)
 #print zed_depth


def navigate():
 global Vx
 global Vy
 global Vz
 global v_yaw
 global v_pitch
 global v_roll
 global Weight_camera_x
 global Weight_camera_y
 global Weight_camera_z
 global Weight_camera_yaw
 global Weight_depth
 global Weight_yaw
 global Weight_pitch
 global Weight_roll
 global required_depth
 global required_pitch
 global required_roll
 global required_yaw
 global moving_towards_object
 global disable_yaw
 global disable_pitch
 global disable_roll
 global disable_Vx
 global disable_Vy
 global disable_Vz
 global required_yaw
 global required_depth
 global disable_red
 global disable_white
 global disable_bin
 global distance_counter 
 global stop_distance_counter
 disable_Vx = 1
 disable_Vy = 0
 disable_Vz = 0
 disable_roll = 0
 disable_pitch = 0
 disable_yaw = 0
 disable_red = 1
 disable_white = 0
 disable_bin = 1
 Weight_camera_x = 0
 Weight_camera_y = 0
 Weight_camera_z = 0
 Weight_camera_yaw = 1
 Weight_depth = 1
 Weight_yaw = 0
 Weight_pitch = 1
 Weight_roll = 0
 if(offset_x<50 and offset_x>-50):
  Vx = 80
 else:
  Vx = 0
 #print 'Vx:' + str(intVx) +' Vy:' + str(Vy) +' Vz:' + str(Vz) +' v_roll:' + str(roll) +' v_pitch:' + str(pitch) +' v_yaw:' + str(yaw) + '  //////  ' + str(offset_x)
 print str(offset_x) + ' / ' + str(yaw) 


def process():
 global Vx
 global Vy
 global Vz
 global error_depth
 global object_detected
 global ip_data_counter
 global v_yaw
 global v_pitch
 global v_roll
 global distance_counter
 global error_yaw
 if(stop_distance_counter==0):
  distance_counter = distance_counter+1
 ip_data_counter  = ip_data_counter + 1
 if(ip_data_counter>1000):
  object_detected = 0
# ......................................................................error calculations
 error_depth = required_depth - depth
 error_yaw_temp = (1)*(required_yaw - yaw)         #in timer maybe?
 error_pitch = (required_pitch - int(pitch))
 error_roll = required_roll - roll
#.......................................................................angle conventions
 if(error_yaw_temp>180):
  error_yaw = error_yaw_temp - 360
 if(error_yaw_temp<-180):
  error_yaw = error_yaw_temp + 360
 if(error_yaw_temp<=180 and error_yaw_temp>=-180):
  error_yaw = error_yaw_temp
 #print error_yaw
#........................................................................navigation cases
 navigate()
#......................................................................error corrections
 if(disable_Vx==0):
  Vx = int(offset_x*Kp_camera_x*Weight_camera_x)
 if(disable_Vy==0):
  Vy = int(offset_y*Kp_camera_y*Weight_camera_y)
 if(disable_Vz==0):
  Vz = int(offset_z*Kp_camera_z*Weight_camera_z + error_depth*Kp_depth*Weight_depth)
 if(disable_pitch==0):
  v_pitch = int(error_pitch*Kp_pitch*Weight_pitch)
 if(disable_yaw==0):
  v_yaw = int(error_yaw*Kp_yaw*Weight_yaw + offset_x*Kp_camera_yaw*Weight_camera_yaw)
 if(disable_roll==0):
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
 #print pub_string
 pub.publish(pub_string)
 rospy.sleep(0.01)


if __name__ == '__main__':
 try:
  listner()
  while True:
   process()
 except KeyboardInterrupt:
  print("Shutting Down")

