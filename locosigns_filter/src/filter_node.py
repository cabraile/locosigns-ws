#!/usr/bin/env python
from modules.filter import *
import rospy
#from std_msgs.msg import Float64, Float64MultiArray
from std_msgs.msg import Header
from locosigns_msg.msg import Scalar, Landmark
from enum import Enum

class FilterNode():

    def velocityCallback(self, velocity):
        predicted = self.filter.predict(velocity.data)
        return

    def landmarkCallback(self, data):
        label = data.label
        dist = data.measured_distance
        psi = data.heading_angle
        self.filter.update(label,dist,psi)
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
            self.state_pub.publish(_pos_msg)
            self.state_var_pub.publish(_pos_var_msg)
            rate.sleep()
        return

    def __init__(self):
        rospy.init_node("filter_node", anonymous=False)

        # Inner modules init
        self.update_rate = 100
        self.Delta_T = 1./self.update_rate
        self.filter = Filter(self.Delta_T, sigma_L = 100.0/3.0, sigma_omega = 1e-1)

        # ROS init
        rospy.Subscriber("sensor/velocity", Scalar, self.velocityCallback)
        rospy.Subscriber("sensor/landmark", Landmark, self.landmarkCallback)
        self.state_pub = rospy.Publisher("state/filter/position", Scalar, queue_size=10)
        self.state_var_pub = rospy.Publisher("state/filter/position_variance", Scalar, queue_size=10)
        return
    
if __name__=="__main__":
    node = FilterNode()
    node.run()

    