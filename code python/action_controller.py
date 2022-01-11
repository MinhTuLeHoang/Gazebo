from os import terminal_size
import rospy
import cv2
import numpy as np
import math
import time
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from beginner_tutorials.msg import Num

# Reading .xml file
import xml.etree.ElementTree as ET


##############################################
##                                          ##
##   This file is made by Minh Tu           ##
##   |----------------------------|         ##
##   |version: 1.0.0              |         ##
##   |Update date: 10-01-2022     |         ##
##   |----------------------------|         ##
##   |Description:                |         ##
##   |   - Depend gazebo clock    |         ##
##   |   - Difficult to ctr h     |         ##
##   |----------------------------|         ##
##                                          ##
##############################################

rospy.init_node("action_topic", disable_signals=True)
pub = rospy.Publisher("/cmd_vel",Twist, queue_size=5)
vel_cmd = Twist()

LIDAR = 0
SIGN = 2
LANE = 1

FREE = 0


state = FREE

last_sign_time = 0
last_lane_time = 0

def callbackFunction(msg):
	print("okela")
	global state

	global last_lane_time
	global last_sign_time

	if(msg.header == LIDAR):
		print("I'm lidar msg")
	elif(msg.header == SIGN):
		sgin_id = msg.base_arg
		# root = ET.parse("rules.xml").getroot()
		# rules_arr = root.find("")
		print("I'm traffic sign")
	elif(state == FREE and msg.header == LANE):
		vel_cmd.linear.x = msg.float_1
		vel_cmd.angular.z = msg.float_2
		print("I'm lane detec")
	# else:
	# 	print("something wrong happens")



# From now on
##############

while not rospy.is_shutdown():
	print("---------------begin---------------")
	lis = rospy.Subscriber("/controller", Num, callbackFunction)
	rospy.spin()
