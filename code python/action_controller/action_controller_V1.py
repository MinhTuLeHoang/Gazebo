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
# import xml.etree.ElementTree as ET
from lxml import etree


##############################################
##                                          ##
##   This file is made by Minh Tu           ##
##   |----------------------------|         ##
##   |version: 1.0.0              |         ##
##   |Update date: 12-01-2022     |         ##
##   |----------------------------|         ##
##   |Description:                |         ##
##   |   - Based on rule xml      |         ##
##   |----------------------------|         ##
##                                          ##
##############################################

rospy.init_node("action_topic", disable_signals=True)
pub = rospy.Publisher("/cmd_vel",Twist, queue_size=5)
vel_cmd = Twist()

tree = etree.parse('/home/minhtulehoang/my_ws/src/beginner_tutorials/src/rules.xml')
root = tree.getroot()

def func_on_shutdown():
	vel_cmd.linear.x = 0
	vel_cmd.angular.z = 0
	pub.publish(vel_cmd)
rospy.on_shutdown(func_on_shutdown)

def Initial_System():
	print("Get started")
	print("Please wait . . .")
	t = 0
	while(t == 0):
		t = rospy.get_time()
	print("Starting complete")
	print("=================")




# Some function
##################
def stop():
	vel_cmd.linear.x = 0
	vel_cmd.angular.z = 0
	pub.publish(vel_cmd)

LIDAR = 0
SIGN = 2
LANE = 1

FREE = 0
BUSY = 1


state = FREE



class MyCar:
	class Speed:
		linear = 0
		angular = 0
	speed = Speed()
myCar = MyCar()
	

def callbackFunction(msg):
	print("okela")
	global state
	global myCar


	if(msg.header == LIDAR):
		print("I'm lidar msg")
	elif(msg.header == SIGN):
		print("I see ", end="")
		print(msg.base_arg)
		
		state = BUSY
		sign_id = msg.base_arg

		global root
		rules = root.xpath("./SIGN/rule[@sign_id=$temp]", temp=sign_id)

		current_rule = ""
		for rule in rules:
			condition = rule.find("condition").text
			if(eval(condition)):
				current_rule = rule
				break

		action = current_rule.find("action").attrib["name"]
		eval(action + "()")
		
		# print("I'm traffic sign")
	elif(state == FREE and msg.header == LANE):
		print("=========================")
		print("Run automatic with: ")
		myCar.speed.linear = msg.float_1
		myCar.speed.angular = msg.float_2
		print("   V_linear = ", end="")
		print(myCar.speed.linear)
		print("   V_angular = ", end="")
		print(myCar.speed.angular)

		vel_cmd.linear.x = msg.float_1
		vel_cmd.angular.z = msg.float_2
		pub.publish(vel_cmd)

	# else:
	# 	print("something wrong happens")



# From now on MAIN function
#############################
Initial_System()

while not rospy.is_shutdown():
	print("---------------begin---------------")
	lis = rospy.Subscriber("/controller", Num, callbackFunction)
	rospy.spin()
