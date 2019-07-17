#!/usr/bin/env python
from modules.filter import *
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from enum import Enum

class FilterNode():

    class Index(Enum):
        LABEL = 0
        MEAS_DISTANCE = 1

    def velocityCallback(self, data):
        
        return

    def landmarkCallback(self, data):
        label = data[Index.LABEL]
        dist = data[Index.MEAS_DISTANCE]
        return

    def run(self):
        rospy.spin()
        return

    def __init__(self):
        rospy.init_node("filter_node", anonymous=False)
        rospy.Subscriber("robot_msgs/velocity", Float64, self.velocityCallback)
        rospy.Subscriber("robot_msgs/landmark", Float64MultiArray, self.landmarkCallback)
        return
    
if __name__=="__main__":
    node = FilterNode()
    node.run()

    