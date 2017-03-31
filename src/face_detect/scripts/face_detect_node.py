#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
"""
import roslib
roslib.load_manifest('face_detect')

import sys
import os

import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import *
from face_detect.msg import Faces


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
    rospy.init_node('face_detect_node', anonymous=True)
    display = rospy.get_param("~display",True)

    def detect_and_draw(imgmsg):
        img = br.imgmsg_to_cv2(imgmsg, "bgr8")
        # allocate temporary images
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        f = Faces()
        k = 0
        faces = face_cascade.detectMultiScale(gray, 1.3, 3)
        for (x,y,w,h) in faces:
            current_roi = RegionOfInterest()
            current_roi.x_offset = x
            current_roi.y_offset = y
            current_roi.height = h
            current_roi.width = w
            current_roi.do_rectify = False
            f.roi.append(current_roi)
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            eyes = eye_cascade.detectMultiScale(roi_gray)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

        faces_roi.publish(f)
        faces_img.publish(br.cv2_to_imgmsg(img))
        if (display):
             cv2.imshow('img',img)
        cv2.waitKey(10)


    faces_roi = rospy.Publisher('~faces_roi', Faces, queue_size=10)
    faces_img = rospy.Publisher('~faces_img', Image, queue_size=10)
    rospy.Subscriber("~image", Image, detect_and_draw)
    rospy.spin()
