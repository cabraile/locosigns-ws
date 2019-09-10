#!/usr/bin/env python
import numpy
import rospy
import tf
from modules.filter import *
from sensor_msgs.msg import Imu as MsgImu
from geometry_msgs.msg import TwistStamped as MsgTwist
from geometry_msgs.msg import PoseStamped as MsgPose
from geometry_msgs.msg import Quaternion
from locosigns_msgs.msg import State as MsgState
from locosigns_msgs.msg import Landmark as MsgLandmark

"""
TODO: ADD DIRECTION DETECTION
TODO: CORRECT VELOCITY USING VISUAL ODOMETRY
"""

class FilterNode():

    # FILTERING
    # =========================================================================

    def predict(self, A, B, u, Q):
        # All filters predict the same way
        for name in self.filter_names:
            filter = self.filters[name]
            filter.predict(A,B,u, Q)
        return

    def update(self, filter, z, h, H, R):
        filter.update(z, h, H, R)
        return

    # =========================================================================

    # ROS COMMUNICATION
    # =========================================================================

    def imuCallback(self, imu_msg):
        a = imu_msg.linear_acceleration.x
        sqrt_Delta_T = numpy.sqrt(self.imu_update_period)
        self.accelerometer_bias += self.accelerometer_random_walk * sqrt_Delta_T
        sigma_accel =   self.accelerometer_noise_density * ( 1./sqrt_Delta_T ) + self.accelerometer_bias
        Q = (self.B.dot( sigma_accel**2. )).dot(self.B.transpose())
        self.predict(self.A, self.B, a, Q)
        self.publish()
        return

    def speedometerCallback(self, speedometer_msg):

        def h(X):
            return numpy.array([
            [X[1,0]]
        ])

        v = speedometer_msg.twist.linear.x

        H = numpy.array([
            [0., 1.]
        ])

        R = numpy.array([
            [(0.3 * v) ** 2.]
        ])

        filters_list = [
            self.filters["velocity_correction"],
            self.filters["complete_correction"]
        ]
        z = numpy.array([
            [v]
        ])
        for filter in filters_list:
            self.update( filter, v, h, H, R )

        return

    def landmarkCallback(self, landmark_msg):
        landmark_curr = landmark_msg
        landmark_prev = None
        if(self.landmark_curr is None):
            self.landmark_curr = landmark_curr
        else:
            landmark_prev = self.landmark_curr
        # for z
        w = landmark_curr.measured_distance
        psi = landmark_curr.heading_angle
        l = landmark_curr.label
        d = w * numpy.cos(psi)
        sigma_depth = ( self.stdev_depth * numpy.cos( self.stdev_angle ) )
        label_prev = -1
        if(landmark_prev is not None):
            label_prev = landmark_prev.label
        # for correction
        h = None
        filters_list = [
            self.filters["landmark_correction"],
            self.filters["complete_correction"]
        ]
        for idx in range(len(filters_list)):
            filter = filters_list[idx]
            # Update global position
            if(landmark_curr.label  != label_prev):
                def h(X):
                    return numpy.array([[X[0,0]]] )
                pos = l - d * self.direction
                H = numpy.array([ 
                    [1., 0. ] 
                ])
                R = numpy.array([  
                    [ self.stdev_landmark ** 2.0 + sigma_depth ** 2 ]  
                ])
                z = numpy.array([
                    [pos]
                ])
                self.update(filter, z, h, H, R)
                    
        self.landmark_states = [ filter.state() for filter in filters_list ] # Stores the state when the landmark was detected - used for the odometry update
        self.landmark_curr = landmark_curr
        return
    
    # =========================================================================

    # NODE STAGES
    # =========================================================================
    
    def publish(self):
        for name in self.filter_names:
            filter = self.filters[name]
            state = filter.state()
            covariance = filter.covariance().reshape((4))

            msg = MsgState()
            msg.header.stamp = rospy.Time.now()
            msg.position = state[0,0]
            msg.velocity = state[1,0]
            msg.covariance = covariance.tolist()
            self.filter_publishers[name].publish(msg)
        return 

    def setup(self):
        self.getROSParam()
        self.initVars()
        self.initFilters()
        self.setCallback()
        return

    # =========================================================================

    # SETUP PROCEDURES
    # =========================================================================

    def getROSParam(self):
        self.stdev_landmark = rospy.get_param("~stdev_landmark") # (meters)^2
        self.stdev_depth    = rospy.get_param("~stdev_depth") # (meters)^2
        self.stdev_angle    = rospy.get_param("~stdev_angle") # (meters)^2
        self.direction      = rospy.get_param("~direction") # -1 (backwards) or 1 (forward)
        self.imu_update_rate    = rospy.get_param("~imu_update_rate") # Hertz
        self.accelerometer_noise_density = rospy.get_param("~accelerometer_noise_density")
        self.accelerometer_random_walk = rospy.get_param("~accelerometer_random_walk")

        # Inner vars for the filter
        self.state_init     = numpy.array([[0.0],[0.0]])
        self.state_cov_init = numpy.array([[1e6, 0.0],[0.0, 1e3]])
        return

    def initVars(self):
        self.imu_update_period  = 1./self.imu_update_rate
        self.landmark_curr      = None
        self.velocity_curr      = 0.
        self.accelerometer_bias = 0.

        # KF
        Delta_T = self.imu_update_period
        self.A = numpy.array([
            [1., self.direction * Delta_T],
            [0., 1.]
        ])
        
        self.B = numpy.array([
            [ self.direction * ((Delta_T)**2.)/2. ],
            [ Delta_T ]
        ])
        return 

    def initFilters(self):
        # Initialize filters
        self.filter_names = [
            "uncorrected",
            "velocity_correction",
            "landmark_correction",
            "complete_correction"
        ]
        self.filters = {}
        self.filter_publishers = {}
        for name in self.filter_names:
            self.filters[name] = Filter(self.state_init, self.state_cov_init)
            self.filter_publishers[name] = rospy.Publisher("vehicle/state/filter/state_{}".format(name), MsgState, queue_size=1)
        return

    def setCallback(self):
        rospy.Subscriber("vehicle/sensor/imu"          ,  MsgImu,      self.imuCallback)
        rospy.Subscriber("vehicle/sensor/speedometer"  ,  MsgTwist,    self.speedometerCallback)
        rospy.Subscriber("vehicle/perception/landmark"     ,  MsgLandmark, self.landmarkCallback)
        return

    # =========================================================================

    # MANAGEMENT
    # =========================================================================
    
    def debug(self, msg):
        rospy.loginfo("[FilterNode] {}".format(msg))
        return
    
    def __init__(self):
        self.setup()
        rospy.spin()
        return

    # =========================================================================

if __name__=="__main__":
    rospy.init_node("filter_node", anonymous=False)
    node = FilterNode()

    
