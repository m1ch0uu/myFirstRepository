#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
"""
import roslib
roslib.load_manifest('face_detect_base')

import sys
import os

import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import *
from face_detect_base.msg import perso_roi_type

min_size = (10, 10)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0
display = True

if __name__ == '__main__':
	opencv_dir = '/usr/share/opencv/haarcascades/';

	face_cascade = cv2.CascadeClassifier(opencv_dir + 'haarcascade_frontalface_default.xml')
	if face_cascade.empty():
		print "Could not find face cascade"
		sys.exit(-1)
	eye_cascade = cv2.CascadeClassifier(opencv_dir + 'haarcascade_eye.xml')
	if eye_cascade.empty():
		print "Could not find eye cascade"
		sys.exit(-1)
	br = CvBridge()
	
	rospy.init_node('facedetect')
	display = rospy.get_param("~display",True)
	pub = rospy.Publisher("Region_Of_Interest", perso_roi_type, queue_size=2)

	def detect_and_draw(imgmsg):
		l = perso_roi_type()
		img = br.imgmsg_to_cv2(imgmsg, "bgr8")
		# allocate temporary images
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		faces = face_cascade.detectMultiScale(gray, 1.3, 3)
		for (x,y,w,h) in faces:
			cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
			roi_gray = gray[y:y+h, x:x+w]
			roi_color = img[y:y+h, x:x+w]
			
			new_elem = RegionOfInterest()
			new_elem.x_offset = x
			new_elem.y_offset = y
			new_elem.width = w
			new_elem.height = y
			l.regions.append(new_elem)
			l.type.append(1)
			
			eyes = eye_cascade.detectMultiScale(roi_gray)
			for (ex,ey,ew,eh) in eyes:
				cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

				new_elem = RegionOfInterest()
				new_elem.x_offset = x
				new_elem.y_offset = y
				new_elem.width = w
				new_elem.height = y
				l.regions.append(new_elem)
				l.type.append(2)

			pub.publish(l)
		
		cv2.imshow('img',img)
		cv2.waitKey(10)

	rospy.Subscriber("~image", sensor_msgs.msg.Image, detect_and_draw)

	rospy.spin()
