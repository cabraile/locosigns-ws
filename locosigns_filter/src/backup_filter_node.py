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
        # Time
        time_curr = rospy.Time.now()
        if(self.last_imu_callback is None):
            self.last_imu_callback = time_curr
            return

        a = imu_msg.linear_acceleration.x
        Delta_T = (time_curr - self.last_imu_callback).to_sec()
        if(Delta_T == 0):
            return
        A = numpy.array([
            [1., Delta_T],
            [0., 1.]
        ])
        B = numpy.array([
            [ ((Delta_T)**2.)/2. ],
            [ Delta_T ]
        ])
        sqrt_Delta_T = numpy.sqrt(Delta_T)
        self.accelerometer_bias += self.accelerometer_random_walk * sqrt_Delta_T
        sigma_accel =   self.accelerometer_noise_density * ( 1./sqrt_Delta_T ) + self.accelerometer_bias
        Q = (B.dot( sigma_accel**2. )).dot(B.transpose())
        self.predict(A, B, a, Q)

        # Time
        self.last_imu_callback = time_curr
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
            [(0.1 * v) ** 2.]
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
        detected_on = rospy.Time.now()
        landmark_prev = None
        if(self.landmark_curr is None):
            self.landmark_curr = landmark_curr
            self.detected_on = detected_on
        else:
            landmark_prev = self.landmark_curr
        Delta_T = 0
        if(landmark_prev is not None):
            Delta_T = (landmark_curr.header.stamp - landmark_prev.header.stamp).to_sec()
            print("LANDMARKS K2={}s | K1={}s".format(landmark_curr.header.stamp.to_sec(), landmark_prev.header.stamp.to_sec()))
        #Delta_T = (detected_on - self.detected_on).to_sec()
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
            # Visual odometry update
            elif(Delta_T > 0):
                s_prev = self.landmark_states[idx][0,0]
                def h(X):
                    X = numpy.array([
                        [s_prev - X[0,0]],
                        [X[1,0]]
                    ])
                    return X
                H = numpy.array([
                    [-1., 0.],
                    [0., 1.]
                ])
                w_prev = landmark_prev.measured_distance
                psi_prev = landmark_prev.heading_angle
                d_prev = w_prev * numpy.cos(psi_prev)
                Delta_S = ( d_prev - d ) * self.direction
                V_z = Delta_S / Delta_T
                z = numpy.array([
                    [Delta_S],
                    [V_z]
                ])
                sigma_Delta_S = 2. * sigma_depth
                sigma_V_z = sigma_Delta_S / Delta_T
                R = numpy.array([
                    [sigma_Delta_S**2, sigma_Delta_S * sigma_V_z],
                    [sigma_Delta_S * sigma_V_z, sigma_V_z ** 2]
                ])
                try:    
                    print("z=[ [{}, {} ] ]^T, DeltaT ={}".format(z[0,0], z[1,0], Delta_T) )
                    self.update(filter, z, h, H, R)
                except numpy.linalg.linalg.LinAlgError:
                    print("================ UPDATE ===================")
                    print("R=([ [{}, {} ] ] | H=([ [{}, {} ]".format(R[0,0], R[0,1], H[0,0], H[0,1]) )
                    print("   [ [{}, {} ] ] |      [{}, {} ]]".format(R[1,0], R[1,1], H[1,0], H[1,1]) )
                    print("State={}".format(filter.covariance()))
                    print("===========================================")
                    
        self.detected_on = detected_on
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

    def run(self):
        self.publish()
        return

    def loop(self):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.run()
            rate.sleep()
        return 

    def setup(self):
        rospy.init_node("filter_node", anonymous=False)
        self.getROSParam()
        self.initVars()
        self.initFilters()
        self.setCallback()
        return

    # =========================================================================

    # SETUP PROCEDURES
    # =========================================================================

    def getROSParam(self):
        self.stdev_landmark = rospy.get_param("stdev_signs") # (meters)^2
        self.stdev_depth    = rospy.get_param("stdev_depth") # (meters)^2
        self.stdev_angle    = rospy.get_param("stdev_angle") # (meters)^2
        self.update_rate    = rospy.get_param("update_rate",100) # Hertz
        self.accelerometer_noise_density = rospy.get_param("accelerometer_noise_density")
        self.accelerometer_random_walk = rospy.get_param("accelerometer_random_walk")

        # Inner vars for the filter
        self.state_init     = numpy.array([[0.0],[0.0]])
        self.state_cov_init = numpy.array([[1e6, 0.0],[0.0, 1e3]])
        return

    def initVars(self):
        self.period             = 1./self.update_rate
        self.landmark_curr      = None
        self.velocity_curr      = 0
        self.direction          = 1 # TODO: detect direction
        self.last_imu_callback  = None
        self.accelerometer_bias = 0
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
            self.filter_publishers[name] = rospy.Publisher("state/filter/state_{}".format(name), MsgState, queue_size=1)
        self.imu_pub = rospy.Publisher("sim_sensor/imu_pose", MsgPose, queue_size=1)
        return

    def setCallback(self):
        rospy.Subscriber("sim_sensors/imu"          ,  MsgImu,      self.imuCallback)
        rospy.Subscriber("sim_sensors/speedometer"  ,  MsgTwist,    self.speedometerCallback)
        rospy.Subscriber("sim_sensors/landmark"     ,  MsgLandmark, self.landmarkCallback)
        return

    # =========================================================================

    # MANAGEMENT
    # =========================================================================
    
    def debug(self, msg):
        rospy.loginfo("[filter_node.py] {}".format(msg))
        return
    
    def __init__(self, run=True):
        self.setup()
        if(run):
            self.loop()
        return

    # =========================================================================

if __name__=="__main__":
    node = FilterNode()

    
