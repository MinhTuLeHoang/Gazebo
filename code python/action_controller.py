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
##   |version: 1.1.0              |         ##
##   |Update date: 12-01-2022     |         ##
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

def most_frequent(List):
	return max(set(List), key = List.count)

SIGN_FPS_MODE = 0 
LANE_FPS_MODE = 1
LIDAR_FPS_MODE = 2
def calculate(mode):
	if (mode == SIGN_FPS_MODE):
		print("")
	elif (mode == LANE_FPS_MODE):
		print("")
	elif (mode == LIDAR_FPS_MODE):
		print("")
	else:
		print("Error at calculate(), wrong parameter !!!")
	


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

last_sign_time = 0
sign_arr = []
temp =root.find("SIGN")
print(temp)
print(temp.tag)
SIGN_ARR_SIZE = int(temp.attrib["arr_size"])
SIGN_ARR_ALIVE_TIME = float(temp.attrib["alive_time"])

l = 0

class MyCar:
	class Speed:
		linear = 0
		angular = 0
	speed = Speed()
	class Fps:
		sign_msg = 0
		lane_msg = 0
	fps = Fps()
myCar = MyCar()
	

def callbackFunction(msg):
	print("okela")
	global state
	global myCar
	global last_sign_time

	global sign_arr
	global l


	if(msg.header == LIDAR):
		print("I'm lidar msg")
	elif(msg.header == SIGN):
		temp = rospy.get_time()
		print(temp - l)
		l = temp
		print("I see ", end="")
		print(msg.base_arg)

		if not sign_arr:	#sign_arr is empty
			last_sign_time = rospy.get_time()
		else:
			current_time = rospy.get_time()
			if(current_time - last_sign_time >= SIGN_ARR_ALIVE_TIME):
				print("time out")
				print(current_time, end=" vs ")
				print(last_sign_time)
				print(len(sign_arr))
				sign_arr = []
				last_sign_time = rospy.get_time()

		
		sign_arr.append(msg.base_arg)
		if(len(sign_arr) >= SIGN_ARR_SIZE):
			print("full")
			print(rospy.get_time(), end=" vs ")
			print(last_sign_time)
			state = BUSY
			sign_id = most_frequent(sign_arr)
			sign_arr = []

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
