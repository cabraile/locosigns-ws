#!/usr/bin/env python
import rospy
from numpy import *
from std_msgs.msg import Header
from locosigns_msg.msg import Scalar, Landmark

# TODO ASSUMES THAT THE ROBOT IS MOVING FORWARD
# TODO ADD NOISE
# TODO ADD PSI
DETECTION_RATE = 0.7

class LandmarkDetector():

    def prepareMessage(self, label, dist, heading_angle):
        _header = Header()
        _header.stamp = rospy.Time.now()
        _landmark_msg = Landmark()
        _landmark_msg.header = _header
        _landmark_msg.label = label
        _landmark_msg.measured_distance = dist
        _landmark_msg.heading_angle = heading_angle
        return _landmark_msg

    def positionCallback(self,msg):
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
        
        new_msg = self.prepareMessage(l_detected, l_dist, 0.0)
        self.publisher.publish(new_msg)
        return

    def __generateLandmarks(self):
        self.landmark_list = [(1000.0 * i)  for i in range(0, 100)]
        self.last_l = None
        return

    def __init__(self):
        self.__generateLandmarks()
        rospy.Subscriber("/state/groundtruth/position", Scalar, self.positionCallback)
        self.publisher = rospy.Publisher("/sensor/landmark", Landmark,queue_size=10)
        rospy.spin()
        return

if __name__ == "__main__":
    rospy.init_node('sim_landmark_detector_node')
    LandmarkDetector()