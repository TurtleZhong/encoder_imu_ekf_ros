#!/usr/bin/env python
# HJ: 2/11/20
# This program performs dead reckoning using the wheel encoders only

import time
import serial
import rospy
import struct
import math
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu
import tf

# state for 3D robot (x,y,z,psi,theta,phi)
x = 0
y = 0
z = 0
psi = 0 # yaw
theta = 0 # pitch
phi = 0 # roll
seq = 0
psi_offset = 0

# record previous state 
ticks_l_prev = 0
tickr_r_prev = 0
first_time_encoder = True
first_time_imu = True
theta_offset = 0

# previous covariance
# update orientation here. use RK3
def imu_callback(data):
	global theta, psi, phi, psi_offset, first_time_imu

	# get orientation data: https://dreamanddead.github.io/2019/04/24/understanding-euler-angles.html
	angles = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w], 'rzyx')
	psi = angles[0]
	theta = angles[1]
	phi = angles[2]

	if (first_time_imu):
		psi_offset = psi
		first_time_imu = False
		return	
	psi = psi - psi_offset	
		
def callbackTicks(data):
	global x, y, z, psi, theta, phi, ticks_l_prev, ticks_r_prev, first_time_encoder, seq
	#rospy.loginfo(rospy.get_caller_id() + "Right %s", data.data)
	if (first_time_encoder):
		ticks_l_prev = data.data[0]
		ticks_r_prev = data.data[1]
		first_time_encoder = False		
		return
		
	# base width (m)
	L = 0.6096
	R = 0.127

	# ticks/m... 1440 ticks per revolution	
	ticks_per_m = 1440/(math.pi*2*R)

	# Distance moved by each wheel
	ticks_l_curr = data.data[0]
	ticks_r_curr = data.data[1]

	# Compute distance moved by each wheel	
	Dl = (ticks_l_curr-ticks_l_prev)/ticks_per_m
	Dr = (ticks_r_curr-ticks_r_prev)/ticks_per_m
	Dc = (Dl+Dr)/2
	
	# update states
	x = x + Dc*math.cos(psi)*math.cos(theta)
	y = y + Dc*math.sin(psi)*math.cos(theta)
	z = z - Dc*math.sin(theta)

	# update previous tick count
	ticks_l_prev = ticks_l_curr
	ticks_r_prev = ticks_r_curr
		
	# odometry publisher
	odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
	odom = Odometry()

	# header
	odom.header.seq = seq
	odom.header.stamp = rospy.Time.now()
	odom.header.frame_id = "odom"

	# pose
	# quaternion created from yaw, pitch, roll. the 'szyx' means rotation applied to moving frame in order z y x (yaw, pitch, roll)
	odom_quat = tf.transformations.quaternion_from_euler(psi, theta, phi, 'rzyx')
	odom.pose.pose = Pose(Point(x, y, z), Quaternion(*odom_quat))
	odom_pub.publish(odom)

	# publish tf
    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, z),
                     tf.transformations.quaternion_from_euler(psi, theta, phi),
                     rospy.Time.now(),
                     "/base_link",
                     "/odom")

	seq += 1


def main():
	# start node
	rospy.init_node('dead_reckoning', anonymous=True)

	# subscribe to encoder
	rospy.Subscriber("wheels", Int32MultiArray, callbackTicks)

	# subscribe to IMU
	rospy.Subscriber("/imu/data", Imu, imu_callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	main()
