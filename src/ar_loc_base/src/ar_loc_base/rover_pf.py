import roslib; roslib.load_manifest('ar_loc_base')
import rospy
from numpy import *
from numpy.linalg import pinv, inv
from math import pi, sin, cos
from geometry_msgs.msg import *
import tf
import bisect
import threading

from rover_kinematics import *

class RoverPF(RoverKinematics):
	def __init__(self, initial_pose, initial_uncertainty):
		RoverKinematics.__init__(self)
		self.initial_uncertainty = initial_uncertainty
		self.lock = threading.Lock()
		self.X = mat(vstack(initial_pose))
		# Initialisation of the particle cloud around the initial position
		self.N = 500
		self.particles = [self.X + self.drawNoise(initial_uncertainty) for i in range(0,self.N)]
		self.pa_pub = rospy.Publisher("~particles",PoseArray,queue_size=1)

	def getRotation(self, theta):
		R = mat(zeros((2,2)))
		R[0,0] = cos(theta); R[0,1] = -sin(theta)
		R[1,0] = sin(theta); R[1,1] = cos(theta)
		return R
	
	# Draw a vector uniformly around [0,0,0], scaled by norm
	def drawNoise(self, norm):
		if type(norm)==list:
			return mat(vstack(norm)*(2*random.rand(3,1)-vstack([1,1,1])))
		else:
			return mat(multiply(norm,((2*random.rand(3,1)-vstack([1,1,1])))))

	def MotionModel (self, X, deltaX, incertitude):
		theta = X[2,0]
		Rtheta = mat([[cos(theta), -sin(theta), 0], 
					  [sin(theta),  cos(theta), 0],
					  [         0,           0, 1]]);
		return X + Rtheta*(deltaX + self.drawNoise(incertitude))
		

	def predict(self, motor_state, drive_cfg, encoder_precision):
		self.lock.acquire()
		# The first time, we need to initialise the state
		if self.first_run:
			self.motor_state.copy(motor_state)
			self.first_run = False
			self.lock.release() 
			self.X = numpy.zeros((3,1))
			return 

		iW = self.prepare_inversion_matrix(drive_cfg)
		S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
		
		deltaX = iW*S
		
		incertitude = encoder_precision

		self.motor_state.copy(motor_state)

		self.particles = [self.MotionModel(Xpart,deltaX, incertitude) for Xpart in self.particles]

		print ("Particles made with %f + %f - %f + %f - %f + %f & i = %f" % (self.X[0,0],self.X[1,0],self.X[2,0],
			deltaX[0,0],deltaX[1,0],deltaX[2,0],incertitude))

		self.lock.release()

	def poids_particule (self, x, Z, L, incertitude):
		X = mat(vstack([x[0,0],x[1,0]])) + self.getRotation(x[2,0])*Z
		err = (X-L)/incertitude
		return exp(-0.5*err.T*err)

	def update_ar(self, Z, L, Uncertainty): 
		self.lock.acquire()
		print "Update: L="+str(L.T)
		# Implement particle filter update using landmarks here
		# Note: the function bisect.bisect_left could be useful to implement
		# the resampling process efficiently
		# TODO

		poids=[self.poids_particule(x,Z,L,Uncertainty) for x in self.particles]
		a = sum (poids)
		p = [x/a for x in poids]
		c = cumsum (p)
		vals = random.rand(len(poids))
		choix = [bisect.bisect_left(c,x) for x in vals]
		self.particles = [self.particles[i] for i in choix]
		
		self.lock.release()

	def update_compass(self, angle, Uncertainty):
		self.lock.acquire()
		print "Update: C="+str(angle)
		# Implement particle filter update using landmarks here
		# Note: the function bisect.bisect_left could be useful to implement
		# the resampling process efficiently
		# TODO


		self.lock.release()

	def updateMean(self):
		X = mat(zeros((3,1)))
		for x in self.particles:
			X += x
		self.X = X / len(self.particles)
		
		return self.X

	def publish(self, pose_pub, target_frame, stamp):
		# Only compute the mean for plotting
		self.updateMean()
		pose = PoseStamped()
		pose.header.frame_id = target_frame
		pose.header.stamp = stamp
		pose.pose.position.x = self.X[0,0]
		pose.pose.position.y = self.X[1,0]
		pose.pose.position.z = 0.0
		Q = tf.transformations.quaternion_from_euler(0, 0, self.X[2,0])
		pose.pose.orientation.x = Q[0]
		pose.pose.orientation.y = Q[1]
		pose.pose.orientation.z = Q[2]
		pose.pose.orientation.w = Q[3]
		pose_pub.publish(pose)

		pa = PoseArray()
		pa.header = pose.header
		for p in self.particles:
			po = Pose()
			po.position.x = p[0,0]
			po.position.y = p[1,0]
			q = tf.transformations.quaternion_from_euler(0, 0, p[2,0])
			po.orientation = Quaternion(*q)
			pa.poses.append(po)
		self.pa_pub.publish(pa)

	def broadcast(self,br, target_frame, stamp):
		br.sendTransform((self.X[0,0], self.X[1,0], 0),
					 tf.transformations.quaternion_from_euler(0, 0, self.X[2,0]),
					 stamp, "/%s/ground"%self.name, target_frame)
		

