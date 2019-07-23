#!/usr/bin/env python
import rospy
from numpy import *
from std_msgs.msg import Header
from locosigns_msg.msg import Scalar

class Velocimeter():

    def velocityCallback(self, msg):
        noise = ( 0.1 * msg.data ) * (2.0 * random.rand() - 1.0)
        new_msg=msg
        new_msg.data = msg.data + noise
        self.publisher.publish(new_msg)
        return

    def __init__(self):
        rospy.Subscriber("/state/groundtruth/velocity", Scalar, self.velocityCallback)
        self.publisher = rospy.Publisher("/sensor/velocity", Scalar,queue_size=10)
        rospy.spin()
        return

if __name__ == "__main__":
    rospy.init_node('sim_velocimeter_node')
    Velocimeter()