#!/usr/bin/python

# Code de Nora et Romain 
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

scale=2.0
vel=0.5

tc.WaitForAuto()
try:
    tc.Wander()

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")


'''
# Code de Gregoire et Remi

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

while True:

	# Start the wait for face task in the background
	w4face = tc.WaitForFace(foreground=False)
	# Prepare a condition so that the following gets executed only until the
	# Region of Interest is found
	tc.addCondition(ConditionIsCompleted("Face detector",tc,w4face))
	try:
		tc.Wander(max_angular_speed=0.5, max_linear_speed=0.5)
	except TaskConditionException, e:
		rospy.loginfo("Path following interrupted on condition: %s" % \
				" or ".join([str(c) for c in e.conditions])) 

rospy.loginfo("Mission completed")
'''
