#!/usr/bin/env python
import rospy
from numpy import *
from std_msgs.msg import Header
from locosigns_msg.msg import Scalar, Landmark

# TODO PLACE THEM INTO THE 3D WORLD
# TODO ADD PSI

class LandmarkDetector():

    class Landmark():
        def __init__(self, position, label):
            self.position = position
            self.label = label
            return
            
    class Landmark3D():
        def __init__(self, x, y, z, label):
            self.position_x = x
            self.position_y = y
            self.position_z = z
            self.label = label
            return

    def _prepareMessage(self, label, dist, heading_angle):
        _header = Header()
        _header.stamp = rospy.Time.now()
        _landmark_msg = Landmark()
        _landmark_msg.header = _header
        _landmark_msg.label = label
        _landmark_msg.measured_distance = dist
        _landmark_msg.heading_angle = heading_angle
        return _landmark_msg

    def _positionCallback(self,msg):
        pos = msg.data
        last_l = self.last_l
        l_detected = None
        l_dist = -1
        # detect
        for l in self.landmark_list:
            dist_true = (l.position - pos) * (1.0 * self.direction)
            if( dist_true > 0.1 and dist_true < 20):
                if(last_l is not None and last_l.label == l.label):
                    continue
                # There is a chance that the landmark is not detected
                _p = self.detection_rate
                will_detect = random.choice([1, 0], 1, p=[_p, 1.0-_p])
                if(not will_detect):
                    break
                l_detected = l
                l_dist = abs(dist_true)
                break 
        # publish
        if(l_detected is None):
            return
        
        new_msg = self._prepareMessage(l_detected.label, l_dist, 0.0)
        self.publisher.publish(new_msg)
        return

    def __generateLandmarks(self):
        self.landmark_list = []
        for i in range(0, 100):
            label = (1000.0 * i)
            noise = (2.0 * random.rand() - 1 ) * self.stdev_landmark * 3.0
            position = label+noise
            lmk = self.Landmark(position, label)
            self.landmark_list.append( lmk )
        self.last_l = None
        return

    def __init__(self):
        # Init ROS and loads parameters
        rospy.init_node('sim_landmark_detector_node')
        self.detection_rate = rospy.get_param("~detection_rate", 0.7)
        self.direction = rospy.get_param("direction")
        self.stdev_landmark = rospy.get_param("stdev_landmark")
        # Place landmarks on the environment
        self.__generateLandmarks()
        # ROS communication
        rospy.Subscriber("/state/groundtruth/position", Scalar, self._positionCallback)
        self.publisher = rospy.Publisher("/sensor/landmark", Landmark,queue_size=10)
        # Loop
        rospy.spin()
        return

if __name__ == "__main__":
    LandmarkDetector()