#!/usr/bin/env python

# - REGULAR MODULES
import os
from numpy import *

# - ROS MODULES
import rospy
import tf

# - ROS MESSAGES
import nav_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.srv import GetModelState
from locosigns_msgs.msg import Scalar, Landmark, Pose

class LandmarkDetector():
    
    # LOOP
    # ==============================
    def poseCallback(self,msg):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        orient = array([orient.x, orient.y, orient.z, orient.w])
        _,_,self.heading = tf.transformations.euler_from_quaternion(orient)
        last_l = self.last_l
        l_detected = None
        l_dist = -1
        heading_angle = 0.0

        # Detect
        idx = 0
        for l in self.landmark_list:
            #if(l.detected):
            #    continue
            norm =  sqrt((l.position_x - pos.x) ** 2.0 + \
                    (l.position_y - pos.y) ** 2.0 + \
                    (l.position_z - pos.z) ** 2.0)
            dist_true = norm
            idx += 1
            if( dist_true > 0.1 and dist_true < 20):
                #if(last_l is not None and last_l.label == l.label):
                #    continue
                # There is a chance that the landmark is not detected
                _p = self.detection_rate
                will_detect = random.choice([1, 0], 1, p=[_p, 1.0-_p])
                if(not will_detect):
                    break
                #rospy.loginfo("--- Detected landmark: {}".format(l_detected.label))

                l_detected = l
                l_dist = abs(dist_true)
                v2 = array([l.position_x - pos.x , l.position_y - pos.y])
                v1 = array([cos(self.heading), sin(self.heading)])
                heading_angle = arccos(dot(v1,v2)/(linalg.norm(v1)*linalg.norm(v2)) )
                l.detected = True
                break 
        
        # Publish if a landmark is detected
        if(l_detected is None):
            return

        new_msg = self.prepareMessage(l_detected.label, l_dist, heading_angle)
        self.publisher.publish(new_msg)
        return
     
    # ==============================

    # SETUP
    # ==============================
           
    class Landmark3D():
        def __init__(self, x, y, z, label):
            self.position_x = x
            self.position_y = y
            self.position_z = z
            self.label = label
            self.detected = False
            return

    def __init__(self):
        # Init ROS and loads parameters
        rospy.init_node("sim_landmark_detector_node")
        self.detection_rate = rospy.get_param("~detection_rate", 1.0)
        self.stdev_landmark = rospy.get_param("~stdev_landmark")

        # Get the list of landmarks on the environment
        self.getLandmarkList()
        self.last_l = None
        
        # ROS communication
        rospy.Subscriber("/base_pose_ground_truth", nav_msgs.msg.Odometry, self.poseCallback)
        self.publisher = rospy.Publisher("/sim_sensors/landmark", Landmark,queue_size=2)

        # Loop
        rospy.spin()
        return

    def prepareMessage(self, label, dist, heading_angle):
        _landmark_msg = Landmark()
        _landmark_msg.header.stamp = rospy.Time.now()
        _landmark_msg.label = label
        _landmark_msg.measured_distance = dist
        _landmark_msg.heading_angle = heading_angle
        return _landmark_msg

    def getLandmarkList(self):
        self.model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.landmark_list = []
        for i in range(1,8+1):
            l_name = "landmark_{}".format(i)
            state = self.model_coordinates(l_name, "")
            x = state.pose.position.x
            y = state.pose.position.y
            z = state.pose.position.z
            label = i * 1000
            self.landmark_list.append( self.Landmark3D(x,y,z,label) )
        return

    # ==============================

if __name__ == "__main__":
    LandmarkDetector()