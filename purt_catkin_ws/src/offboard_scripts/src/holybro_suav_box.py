#!/usr/bin/env python3

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
#from nav_msgs.msg import Odometry

current_x = 0.0
current_y = 0.0
current_z = -1.0

pose = PoseStamped()


def callback(data):
	global current_x
	global current_y
	global current_z

	#rospy.loginfo("I am in callback function")

	current_x = data.pose.position.x
	current_y = data.pose.position.y
	current_z = data.pose.position.z

	#rospy.loginfo("x-coordinates : %f", current_x)
	#rospy.loginfo("y-coordinates : %f", current_y)
	#rospy.loginfo("z-coordinates : %f", current_z)


	
if __name__== '__main__':


	#Initialize ros node
	rospy.init_node('holybro_suas_hover', anonymous=True)
	rate = rospy.Rate(25.0)
	
	#Subscribe to qualisys topic
	#rospy.Subscriber("/qualisys/hb1/pose", PoseStamped, callback)
	
	#rospy.loginfo("x-coordinates out : %f", current_x)
	#rospy.loginfo("y-coordinates out : %f", current_y)
	#rospy.loginfo("z-coordinates out : %f", current_z)
	


	#Arm all vehicles
	arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	arming_client(True)

	print("Armed the vehicles")

	
	#Send a few starting setpoints
	for j in range(200):
		pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		pose.pose.position.x = current_x #change this
		pose.pose.position.y = current_y	#change this
		pose.pose.position.z = current_z	#change this
		
		pos_pub.publish(pose)
		rate.sleep()
	

	#Set off board flight mode
	set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
	set_mode_client(base_mode=0, custom_mode="OFFBOARD")
	print('Finished changing flight modes')

	
	print("Starting takeoff")
	#Takeoff altitude
	for j in range(150):

		#Subscribe to qualisys topic
		#rospy.Subscriber("/qualisys/hb1/pose", PoseStamped, callback)

		pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x = current_x #change this
		pose.pose.position.y = current_y #change this
		pose.pose.position.z = 2.0 #fix this

		pos_pub.publish(pose)
		rate.sleep()

	print("Finished Takeoff")
	print("Start box pattern")

	for j in range(200):

		#Subscribe to qualisys topic
		#rospy.Subscriber("/qualisys/hb1/pose", PoseStamped, callback)

		pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x = current_x #change this
		pose.pose.position.y = -3.0 #change this
		pose.pose.position.z = 2.0 #fix this

		pos_pub.publish(pose)
		rate.sleep()

	for j in range(200):
		pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x = -2.0 #change this
		pose.pose.position.y = -3.0 #change this
		pose.pose.position.z = 2.0 #decrease this

		pos_pub.publish(pose)
		rate.sleep()

	for j in range(200):
		pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x = -2.0 #change this
		pose.pose.position.y = 0.0 #change this
		pose.pose.position.z = 2.0 #decrease this

		pos_pub.publish(pose)
		rate.sleep()

	for j in range(200):
		pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x = 0.0 #change this
		pose.pose.position.y = 0.0 #change this
		pose.pose.position.z = 2.0 #decrease this

		pos_pub.publish(pose)
		rate.sleep()

	print("Finished box pattern")

	print("Start landing")

	for j in range(75):
		pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x = 0.0 #change this
		pose.pose.position.y = 0.0 #change this
		pose.pose.position.z = 0.0 #decrease this

		pos_pub.publish(pose)
		rate.sleep()
	
	print("Finished Landing")
	
	arming_client(False)

	print("Disarmed the vehicles")
	
	print("Mission complete")
	