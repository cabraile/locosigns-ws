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
        rospy.spin()
        return

    def __init__(self):
        # Inner modules init
        self.filter = Filter() # TODO: ADD PARAMETRIZATION CONF

        # ROS init
        rospy.init_node("filter_node", anonymous=False)
        rospy.Subscriber("robot_msgs/velocity", Float64, self.velocityCallback)
        rospy.Subscriber("robot_msgs/landmark", Float64MultiArray, self.landmarkCallback)
        return
    
if __name__=="__main__":
    node = FilterNode()
    node.run()

    