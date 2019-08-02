#!/usr/bin/env python
from modules.filter import *
import rospy
#from std_msgs.msg import Float64, Float64MultiArray
from std_msgs.msg import Header
from locosigns_msg.msg import Scalar, Landmark
from enum import Enum

class FilterNode():

    def velocityCallback(self, velocity_msg):
        self.velocity = velocity_msg.data
        return

    def landmarkCallback(self, landmark_msg):
        self.landmark = landmark_msg
        return

    def prepareMessages(self):
        _pos_msg = Scalar()
        _pos_var_msg = Scalar()
        
        _pos_msg.header.stamp = rospy.get_rostime()
        _pos_var_msg.header.stamp = rospy.get_rostime()

        _pos_msg.data = self.filter.S
        _pos_var_msg.data = self.filter.P
        return _pos_msg, _pos_var_msg

    def run(self):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            _pos_msg, _pos_var_msg = self.prepareMessages()
            self.filter.predict(self.velocity)
            if(self.landmark is not None):
                label = self.landmark.label
                dist = self.landmark.measured_distance
                psi = self.landmark.heading_angle
                self.filter.update(label,dist,psi)
                self.landmark = None
            self.state_pub.publish(_pos_msg)
            self.state_var_pub.publish(_pos_var_msg)
            rate.sleep()
        return

    def __init__(self):
        rospy.init_node("filter_node", anonymous=False)
        # Inner modules init
        sigma_L = rospy.get_param("~stdev_landmark", 100./3.)
        print(sigma_L)
        self.update_rate = 2000
        self.Delta_T = 1./self.update_rate
        self.landmark = None
        self.velocity = 0
        self.filter = Filter(self.Delta_T, sigma_L = sigma_L, sigma_omega = 1e-1)
        # ROS init
        rospy.Subscriber("sim_sensor/velocity", Scalar, self.velocityCallback)
        rospy.Subscriber("sensor/landmark", Landmark, self.landmarkCallback)
        self.state_pub = rospy.Publisher("state/filter/position", Scalar, queue_size=1)
        self.state_var_pub = rospy.Publisher("state/filter/position_variance", Scalar, queue_size=1)
        return
    
if __name__=="__main__":
    node = FilterNode()
    node.run()

    