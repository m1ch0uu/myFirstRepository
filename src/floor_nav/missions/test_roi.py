#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
import random
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)
random.seed()

tc.WaitForAuto()

while True:
	# Start the wait for roi task in the background
	w4roi = tc.WaitForFace(foreground=False)
	# Prepare a condition so that the following gets executed only until the 
	# Region of Interest is found
	tc.addCondition(ConditionIsCompleted("Face detector",tc,w4roi))

	rospy.loginfo("Pas d'erreur en vue !")
	try:
		tc.Wander(max_angular_speed=0.5, max_linear_speed=0.5 	)
	except TaskConditionException, e:
		t = random.uniform(-2,2)
		rospy.loginfo("t: %f", t)
		tc.StareAtFace(new_heading=t, k_theta=1., max_angular_velocity=0.5)
		#rospy.loginfo("Exception caught: " + str(e))


rospy.loginfo("Mission completed")


