#!/usr/bin/env python
import rospy
from numpy import *
from std_msgs.msg import Float64, Float64MultiArray

# TODO ASSUMES THAT THE ROBOT IS MOVING FORWARD
# TODO ADD NOISE

DETECTION_RATE = 0.7

class LandmarkDetector():

    def positionCallback(msg):
        pos = msg.data
        last_l = self.last_l
        l_detected = None
        l_dist = -1
        # detect
        for l in self.landmark_list:
            dist_true = l - pos
            if( dist_true > 0.1 and dist_true < 20):
                if(last_l is not None and last_l == l):
                    continue
                # There is a chance that the landmark is not detected
                _p = DETECTION_RATE
                will_detect = random.choice([1, 0], 1, p=[_p, 1.0-_p])
                if(not will_detect):
                    break
                l_detected = l
                l_dist = abs(dist_true)
                break 
        # publish
        if(l_detected is None):
            return
        new_msg = Float64MultiArray([l_detected, l_dist])
        self.publisher.publish(new_msg)
        return

    def __generateLandmarks(self):
        self.landmark_list = [1000 * i for i in range(1, 101)]
        self.last_l = None
        return

    def __init__(self):
        self.__generateLandmarks()
        rospy.Subscriber("/state/groundtruth/position", Float64, self.positionCallback)
        self.publisher = rospy.Publisher("/sensor/landmark", Float64MultiArray,queue_size=10)
        rospy.spin()
        return

if __name__ == "__main__":
    rospy.init_node('sim_landmark_detector_node')
    LandmarkDetector()