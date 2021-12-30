#! /usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan

from beginner_tutorials.msg import Num

import numpy as np
import time




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




#action
######################
def stop():
	pub = rospy.Publisher('chatter', Num, queue_size=20)
	num_msg = Num()
	num_msg.header = 5
	num_msg.base_arg = 0
	num_msg.int_1 = 0		#alpha in change_lane.py
	num_msg.float_1 = 0		#delta_d in chane_lane.py
	num_msg.float_2 = 0		#V_a   in change_lane.py
	pub.publish(num_msg)


def change_lane_toLeft():
	pub = rospy.Publisher('chatter', Num, queue_size=20)
	num_msg = Num()
	num_msg.header = 5
	num_msg.base_arg = 1
	num_msg.int_1 = 15		#alpha in change_lane.py
	num_msg.float_1 = 0.4		#delta_d in chane_lane.py
	num_msg.float_2 = 0.15		#V_a   in change_lane.py
	pub.publish(num_msg)


def change_lane_toRight():
	pub = rospy.Publisher('chatter', Num, queue_size=20)
	num_msg = Num()
	num_msg.header = 5
	num_msg.base_arg = 2
	num_msg.int_1 = 15		#alpha in change_lane.py
	num_msg.float_1 = 0.4		#delta_d in chane_lane.py
	num_msg.float_2 = 0.15		#V_a   in change_lane.py
	pub.publish(num_msg)





#Lidar callback func
##############################
def callbackFunction(data):
	data_ranges = data.ranges
	f = front_side(data_ranges)
	if(f == BLOCKING):
		l = left_side(data_ranges)
		r = right_side(data_ranges)
		if(l == BLOCKING and r == BLOCKING):
			stop()
		elif(l == BLOCKING):
			change_lane_toRight()
		elif(r == BLOCKING):
			change_lane_toLeft()
	elif(f == WARNING):
		stop()
	elif(f == COLLISION):
		stop()
	print ("done")




#______MAIN______#
#----------------#
while not rospy.is_shutdown():
	print("---------------begin---------------")
	rospy.init_node("lidar_topic")
	lis = rospy.Subscriber("/scan", LaserScan, callbackFunction)
	rospy.spin()












