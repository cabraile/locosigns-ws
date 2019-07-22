#!/usr/bin/env python
from modules.filter import *
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from enum import Enum

class FilterNode():

    class INDICES(Enum):
        LABEL = 0
        MEAS_DISTANCE = 1
        HEADING_ANGLE = 2

    def velocityCallback(self, velocity):
        self.filter.predict(velocity)
        return

    def landmarkCallback(self, data):
        label = data[INDICES.LABEL]
        dist = data[INDICES.MEAS_DISTANCE]
        psi = data[INDICES.HEADING_ANGLE]
        self.filter.update(label,dist,psi)
        return

    def run(self):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.state_pub.publish(self.filter.S)
            self.state_var_pub.publish(self.filter.P)
            rate.sleep()
        return

    def __init__(self):
        rospy.init_node("filter_node", anonymous=False)

        # Inner modules init
        self.update_rate = 100
        self.Delta_T = 1./self.update_rate
        self.filter = Filter(self.Delta_T, sigma_L = 0.0, sigma_omega = 0.0)

        # ROS init
        rospy.Subscriber("robot_msgs/velocity", Float64, self.velocityCallback)
        rospy.Subscriber("robot_msgs/landmark", Float64MultiArray, self.landmarkCallback)
        self.state_pub = rospy.Publisher("state/filter/position", Float64)
        self.state_var_pub = rospy.Publisher("state/filter/position_variance", Float64)
        return
    
if __name__=="__main__":
    node = FilterNode()
    node.run()

    