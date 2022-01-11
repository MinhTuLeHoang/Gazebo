from os import terminal_size
import rospy
import cv2
import numpy as np
import math
import time
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock


################################################
##                                            ##
##  This file is made by Minh Tu              ##
##  |----------------------------|            ##
##  |version: 1.0.0              |            ##
##  |Update date: 29-12-2021     |            ##
##  |----------------------------|            ##
##  |Description:                |            ##
##  |   - Depend gazebo clock    |            ##
##  |   - Difficult to ctr h     |            ##
##  |----------------------------|            ##
##                                            ##
################################################


rospy.init_node("change_lane_topic", disable_signals=True)
pub = rospy.Publisher("/cmd_vel",Twist, queue_size=5)
vel_cmd = Twist()


def func_on_shutdown():
	vel_cmd.linear.x = 0
	vel_cmd.angular.z = 0
	pub.publish(vel_cmd)

rospy.on_shutdown(func_on_shutdown)


DELAY_TIME = 0.4


base_arg = 2		#1-left, 2-right
delta_d = 0.45		#do rong chuyen lan
v_linear = 0.1		#v thang (m/s)
v_angular = 0.15	#V goc (rad/s)
alpha = 20		#goc sau cua


r = v_linear / (v_angular * math.pi)

t1 = (alpha / 180) * (math.pi / v_angular)
d1 = 2*r * math.sin( math.radians(alpha/2) ) * math.sin( math.radians(alpha/2) )
h1 = d1 / math.tan( math.radians(alpha/2) )

d2 = delta_d - 2*d1
h2 = d2 / math.tan( math.radians(alpha) )
t2 = d2 / (v_linear * math.sin( math.radians(alpha) ))

h = 2*h1 + h2


print("##########################")
print("r: ", end=""), print(r)
print("")
print("d1: ", end=""), print(d1)
print("h1: ", end=""), print(h1)
print("t1: ", end=""), print(t1)
print("")
print("d2: ", end=""), print(d2)
print("h2: ", end=""), print(h2)
print("t2: ", end=""), print(t2)
print("")
print("h: ", end=""), print(h)
print("##########################")

direction = 0
if (base_arg == 1):
	direction = 1
elif (base_arg == 2):
	direction = -1





##################################
##             MAIN             ##
##################################





print("---STATE 1---")
state1_time = 0
while(state1_time == 0):
	state1_time = rospy.get_time()
print("Begin time:", end=""), print(state1_time)
print(time.time())
print("   ---   ")

vel_cmd.linear.x = v_linear
vel_cmd.angular.z = direction * v_angular

while(True):
	pub.publish (vel_cmd)
	t = rospy.get_time() - state1_time
	if(t >= (t1 + DELAY_TIME)):
		break;
	pub.publish (vel_cmd)




print("---STATE 2---")
state2_time = rospy.get_time()
print("Begin time:", end=""), print(state2_time)
print(time.time())

vel_cmd.linear.x = v_linear
vel_cmd.angular.z = 0

while(True):
	pub.publish (vel_cmd)
	t = rospy.get_time() - state2_time
	if(t >= t2):
		break;
	pub.publish (vel_cmd)





print("---STATE 3---")
state3_time = rospy.get_time()
print("Begin time:", end=""), print(state3_time)
print(time.time())

vel_cmd.linear.x = v_linear
vel_cmd.angular.z = (-1) * direction * v_angular

while(True):
	pub.publish (vel_cmd)
	t = rospy.get_time() - state3_time
	if(t >= t1):
		break;
	pub.publish (vel_cmd)





print("---DONE---")
done = rospy.get_time()
print("Begin time:", end=""), print(done)
print(time.time())

vel_cmd.linear.x = 0
vel_cmd.angular.z = 0

while(True):
	t = rospy.get_time() - done
	if(t >= 2):
		break;
	pub.publish (vel_cmd)


