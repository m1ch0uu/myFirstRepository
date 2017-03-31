import roslib; roslib.load_manifest('ar_loc_base')
import rospy
from numpy import *
from numpy.linalg import pinv, inv
from math import pi, sin, cos
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf
import threading

from rover_kinematics import *

class RoverKF(RoverKinematics):
    def __init__(self, initial_pose, initial_uncertainty):
        RoverKinematics.__init__(self)
        self.lock = threading.Lock()
        self.X = mat(vstack(initial_pose))
        self.P = mat(diag(initial_uncertainty))
        self.ellipse_pub = rospy.Publisher("~ellipse",Marker,queue_size=1)
        self.pose_with_cov_pub = rospy.Publisher("~pose_with_covariance",PoseWithCovarianceStamped,queue_size=1)
        self.Q = eye(3)
    def getRotation(self, theta):
        R = mat(zeros((2,2)))
        R[0,0] = cos(theta); R[0,1] = -sin(theta)
        R[1,0] = sin(theta); R[1,1] = cos(theta)
        return R
    
    def predict(self, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return 
        # Prepare odometry matrices (check rover_odo.py for usage)
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
        self.motor_state.copy(motor_state)
    
        t = self.X[2,0]

        RotateMatrix = mat([[cos(t), -sin(t), 0], 
                            [sin(t),  cos(t), 0],
                            [0, 0, 1]]);

        DeltaX = iW*S
        
        self.X = self.X + RotateMatrix*DeltaX
        
        A = mat([[1,0, -sin(t)*DeltaX[0,0]-cos(t)*DeltaX[1,0]],
                 [0,1,  cos(t)*DeltaX[0,0]-sin(t)*DeltaX[1,0]],
                 [0,0,  1 ]])

        self.P = A * self.P * A.T + self.Q

        self.lock.release()

    def update_ar(self, Z, L, uncertainty):
        self.lock.acquire()
        print "Update: L="+str(L.T)+" X="+str(self.X.T)
        
        R = mat(diag([uncertainty,uncertainty]))
        theta = self.X[2,0]
        Rmt = self.getRotation(-theta)

        z = Rmt*mat([L[0,0]-self.X[0,0], L[1,0]-self.X[1,0]]).T
        H = mat([[-cos(theta), -sin(theta), -(L[0,0]-self.X[0,0])*sin(theta) + (L[1,0]-self.X[1,0])*cos(theta)],
                 [ sin(theta), -cos(theta), -(L[0,0]-self.X[0,0])*cos(theta) - (L[1,0]-self.X[1,0])*sin(theta)]])
        S = H * self.P * H.T + R
        Kk = self.P * H.T * inv(S)

        self.X = self.X + Kk *(Z - z)
        self.P = (eye(3) - Kk * H) * self.P

        print "update \nX : " + str(self.X) + "\n P : " + str(self.P) + "\n"
        self.lock.release()

    def update_compass(self, Z, uncertainty):
        self.lock.acquire()
        print "Update: S="+str(Z)+" X="+str(self.X.T)
        # Implement kalman update using compass here
        # TODO
        # self.X = 
        # self.P = 
        self.lock.release()
        return 

    # this publishes the pose but also the pose with covariance and the error ellipse in rviz
    def publish(self, pose_pub, target_frame, stamp):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = stamp
        pose.pose.pose.position.x = self.X[0,0]
        pose.pose.pose.position.y = self.X[1,0]
        pose.pose.pose.position.z = 0.0
        Q = tf.transformations.quaternion_from_euler(0, 0, self.X[2,0])
        pose.pose.pose.orientation.x = Q[0]
        pose.pose.pose.orientation.y = Q[1]
        pose.pose.pose.orientation.z = Q[2]
        pose.pose.pose.orientation.w = Q[3]
        psub = PoseStamped()
        psub.header = pose.header
        psub.pose = pose.pose.pose
        pose_pub.publish(psub)
        C = [0]*36
        C[ 0] = self.P[0,0]; C[ 1] = self.P[0,1]; C[ 5] = self.P[0,2]
        C[ 6] = self.P[1,0]; C[ 7] = self.P[1,1]; C[11] = self.P[1,2]
        C[30] = self.P[2,0]; C[31] = self.P[2,1]; C[35] = self.P[2,2]
        pose.pose.covariance = C
        self.pose_with_cov_pub.publish(pose)
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose.pose
        marker.scale.x = 3*sqrt(self.P[0,0])
        marker.scale.y = 3*sqrt(self.P[1,1]);
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        self.ellipse_pub.publish(marker)

    # broadcast the estimated transform
    def broadcast(self,br, target_frame, stamp):
        br.sendTransform((self.X[0,0], self.X[1,0], 0),
                     tf.transformations.quaternion_from_euler(0, 0, self.X[2,0]),
                     stamp, "/%s/ground"%self.name, target_frame)
        

