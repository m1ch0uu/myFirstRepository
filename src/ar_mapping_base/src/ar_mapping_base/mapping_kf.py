import roslib; roslib.load_manifest('ar_mapping_base')
import rospy
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
import tf
import threading

import rover_driver_base
from rover_driver_base.rover_kinematics import *

class Landmark:
    def __init__(self, Z, X , R):
        self.X = X

        t = X[2,0]
        
        RotateMatrix = mat([[cos(t), -sin(t)],[sin(t), cos(t)]])
        
        self.L = mat([X[1,0],X[2,0]] + dot(RotateMatrix, Z))
        self.P = R

        print "Init Landmark [" + str(self.L[0,0]) + ", " + str(self.L[1,0]) + "]" 

    def update(self,Z, X, R):
        self.X = X
        self.P = R

        t = X[2,0]

        RotateMatrix = mat([[cos(t), -sin(t)],[sin(t), cos(t)]])

        self.L = (mat(mat([X[0,0], X[1,0]]) + mat((RotateMatrix * Z).T))).T

        print "Update Landmark " + str(self.L.T)

        return
        
class MappingKF:
    def __init__(self):
        self.lock = threading.Lock()
        self.marker_list = {}
        self.marker_pub = rospy.Publisher("~landmarks",MarkerArray,queue_size=1)

    def update_ar(self, Z, X, Id, uncertainty):
        self.lock.acquire()
        #print "Update: Z="+str(Z.T)+" X="+str(X.T)+" Id="+str(Id)
        R = mat(diag([uncertainty,uncertainty]))
        if Id in self.marker_list.keys():
            self.marker_list[Id].update(Z,X,R)
        else:
            self.marker_list[Id] = Landmark(Z,X,R)
        self.lock.release()


    def publish(self, target_frame, timestamp):
        ma = MarkerArray()
        for id in self.marker_list:
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0,0]
            marker.pose.position.y = Lkf.L[1,0]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.x = max(3*sqrt(Lkf.P[0,0]),0.05)
            marker.scale.y = max(3*sqrt(Lkf.P[1,1]),0.05)
            marker.scale.z = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime.secs=3.0;
            ma.markers.append(marker)
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = 1000+id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0,0]
            marker.pose.position.y = Lkf.L[1,0]
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.text = str(id)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.lifetime.secs=3.0;
            ma.markers.append(marker)
        self.marker_pub.publish(ma)

