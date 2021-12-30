#! /usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan



import numpy as np
import time



#def publish(traffic_sign):
#	pub = rospy.Publisher('chatter', custom_msg, queue_size=10)
	
#	customMsg = custom_msg()
#	customMsg.name = 1
#	customMsg.num1 = traffic_sign
#	customMsg.num2 = -1
	
#	pub.publish(customMsg)





# GLOBAL VARIABLE
########################
NON_BLOCKING = 0
BLOCKING = 1
WARNING = 2	#nearly collision
COLLISION = 3


# Check vat can
####################
def front_side(arr):
	if(arr[0] < 1.65 and arr[1] < 1.65 and arr[359] < 1.65 and arr[358] < 1.65):
		return BLOCKING
	elif(arr[0] < 0.4 and arr[1] < 0.4 and arr[359] < 0.4 and arr[358] < 0.4):
		return WARNING
	elif(arr[0] < 0.25 and arr[1] < 0.25 and arr[359] < 0.25 and arr[358] < 0.25):
		return COLLISION
	else:
		return NON_BLOCKING


def left_side(arr):
	if(arr[15] < 1.85 and arr[16] < 1.85 and arr[17] < 1.85):
		return BLOCKING
	else:
		return NON_BLOCKING


def right_side(arr):
	if(arr[341] < 1.85 and arr[342] < 1.85 and arr[343] < 1.85):
		return BLOCKING
	else:
		return NON_BLOCKING








#Lidar callback func
##############################
def callbackFunction(data):
	ranges = data.ranges
	for i in range(len(ranges)):
		if(i<=35 or i >= 325):
			print("[", end =" ")
			print(i, end =" ")
			print("] = ", end =" ")
			print(ranges[i])
	f = front_side(ranges)
	l = left_side(ranges)
	r = right_side(ranges)
	print("left_side: "), print(l)
	print("front_side: "), print(f)
	print("right_side: "), print(r)
	print ("done")




#______MAIN______#
#----------------#
while not rospy.is_shutdown():
	print("---------------begin---------------")
	rospy.init_node("lidar_topic")
	lis = rospy.Subscriber("/scan", LaserScan, callbackFunction)
	rospy.spin()












