#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from gazebo_msgs.msg import ModelState
import numpy as np
import time

state = ModelState()
state.model_name = 'iris0'



def mocap_data_pub():
    rospy.init_node('mocap_data_pub', anonymous=True)
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    rate = rospy.Rate(100) 
    
    
    def callback(data):            
        state.pose.position.x = data.pose.position.x
        state.pose.position.y = data.pose.position.y
        state.pose.position.z = data.pose.position.z
        state.pose.orientation.x = data.pose.orientation.x
        state.pose.orientation.y = data.pose.orientation.y
        state.pose.orientation.z = data.pose.orientation.z
        state.pose.orientation.w = data.pose.orientation.w
        print("X Coordinate :",data.pose.position.x)
        print("y Coordinate :",data.pose.position.y)
        print("z Coordinate :",data.pose.position.z)
        

    rospy.Subscriber("/qualisys_node/cf0/pose", PoseStamped, callback)

    while not rospy.is_shutdown():
        pub.publish(state)
        rate.sleep()        

if __name__ == '__main__':
    try:
        mocap_data_pub()
    except rospy.ROSInterruptException:
        pass