#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

scale = 10
k_a = 3
vel = k_a/3
k_b =k_a/16

tc.WaitForAuto()
try:
	tc.GoToPose(goal_x=-scale,goal_y=-scale,goal_theta=-pi/4,k_alpha=k_a, k_r=vel, k_beta=k_b, smart=1, dist_threshold=2, angle_threshold=0.65)
	tc.Wait(duration=1.0)
	tc.GoToPose(goal_x=scale,goal_y=scale,goal_theta=pi/2,k_alpha = k_a, k_r=vel, k_beta=k_b, smart=1, dist_threshold=0.2, angle_threshold=0.5)
	tc.Wait(duration=1.0) 


except TaskException, e:
	rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
	tc.SetManual()


rospy.loginfo("Mission completed")
