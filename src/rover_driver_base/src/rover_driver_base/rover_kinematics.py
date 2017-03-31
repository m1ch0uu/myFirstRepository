#!/usr/bin/env python
import roslib; roslib.load_manifest('rover_driver_base')
import rospy
from geometry_msgs.msg import Twist
import numpy
from numpy.linalg import pinv
from numpy import dot
from numpy import matmul
from math import atan2, hypot, pi, cos, sin, degrees, radians

prefix=["FL","FR","CL","CR","RL","RR"]

class RoverMotors:
	def __init__(self):
		self.steering={}
		self.drive={}
		for k in prefix:
			self.steering[k]=0.0
			self.drive[k]=0.0
	def copy(self,value):
		for k in prefix:
			self.steering[k]=value.steering[k]
			self.drive[k]=value.drive[k]

class DriveConfiguration:
	def __init__(self,radius,x,y,z):
		self.x = x
		self.y = y
		self.z = z
		self.radius = radius


class RoverKinematics:
	def __init__(self):
		self.X = numpy.asmatrix(numpy.zeros((3,1)))
		self.motor_state = RoverMotors()
		self.first_run = True
		self.sens = 1

	def twist_to_motors(self, twist, drive_cfg, skidsteer=False, speed=5.0, rotate=1.5):
		motors = RoverMotors()
		# print "-"*32
		if skidsteer:
			for k in drive_cfg.keys():
				dist = twist.linear.x - rotate*twist.angular.z*drive_cfg[k].y
				motors.steering[k] = 0
				motors.drive[k] = speed * drive / drive_cfg[k].radius

		else:
			for k in drive_cfg.keys():
				dist_x = twist.linear.x - twist.angular.z*drive_cfg[k].y
				dist_y = twist.linear.y + twist.angular.z*drive_cfg[k].x
				motors.steering[k] = atan2(dist_y,dist_x) 
				motors.drive[k] = speed * hypot(dist_y,dist_x)
		return motors

	def handler (self, a, b):
		c = ((a+b/2)%b) - b/2
		return c

	def integrate_odometry(self, motor_state, drive_cfg, skidsteer=False):
		if self.first_run:
			self.motor_state.copy(motor_state)
			self.first_run = False
			self.number_intodom = 0
			self.entraxe = drive_cfg["CL"].y - drive_cfg["CR"].y

			self.X[0,0] = 0
			self.X[1,0] = 0
			self.X[2,0] = 0

			return self.X

		if skidsteer:
			rotationLeft = (motor_state.drive["CL"] - self.motor_state.drive["CL"])
			rotationRight = (motor_state.drive["CR"] - self.motor_state.drive["CR"])
			rotation = (rotationLeft - rotationRight)
			distance = ((rotationLeft + rotationRight) / 2 ) * drive_cfg["CL"].radius

			dTheta = (rotation / self.entraxe)
			dX = distance * cos((2*self.X[2,0] + dTheta)/2)
			dY = distance * sin((2*self.X[2,0] + dTheta)/2)

			self.X[0,0] += dX
			self.X[1,0] += dY
			self.X[2,0] += dTheta
			self.X[2,0] = self.X[2,0] % (2 * pi)
		
		else:
			A = numpy.zeros((len(prefix)*2,3))
			for i in range(len(prefix)):
				A[2*i,0] = 1
				A[2*i,2] = -drive_cfg[prefix[i]].y

				A[2*i+1,1] = 1
				A[2*i+1,2] = drive_cfg[prefix[i]].x 
			A = pinv(A)

			S = numpy.zeros((len(prefix)*2,1))
			for i in range(len(prefix)):
				k = prefix[i]
				S[2*i,0] = self.handler(motor_state.drive[k] - self.motor_state.drive[k], 2*pi)*2*drive_cfg[prefix[i]].radius*cos(motor_state.steering[k])
				S[2*i+1,0] = self.handler(motor_state.drive[k] - self.motor_state.drive[k], 2*pi)*2*drive_cfg[prefix[i]].radius*sin(motor_state.steering[k])

			Ds = numpy.matmul(A, S)

			self.X[0,0] += Ds[0,0] * cos (self.X[2,0]) + Ds[1,0] * sin(self.X[2,0])
			self.X[1,0] += - Ds[0,0] * sin (self.X[2,0]) + Ds[1,0] * cos(self.X[2,0])
			self.X[2,0] += Ds[2,0] 
			self.X[2,0] = self.handler(self.X[2,0],2*pi)

		self.number_intodom += 1
		if self.number_intodom % 5 == 0:
			print ("Position (%f, %f, %f) - motor_state.drive : %f" % (self.X[0,0], self.X[1,0], self.X[2,0], motor_state.drive["CR"]))
		
		self.motor_state.copy(motor_state)

		return self.X