#!/usr/bin/env python
import rospy
from numpy import *
from geometry_msgs.msg import PointStamped
from locosigns_msgs.msg import State
import nav_msgs.msg

class ErrorNode():

    def completeCallback(self, msg):
        self.positions["complete_correction"] = msg.position
        return

    def velocityCallback(self,msg):
        self.positions["velocity_correction"] = msg.position
        return

    def landmarkCallback(self,msg):
        self.positions["landmark_correction"] = msg.position
        return

    def uncorrectedCallback(self,msg):
        self.positions["uncorrected"] = msg.position
        return

    def groundtruthCallback(self, msg):
        self.positions["groundtruth"] = msg.point.x
        return

    def publish(self, name):
        if(self.positions["groundtruth"] is None or self.positions[name] is None):
            return False
        error = abs(self.positions["groundtruth"] - self.positions[name] )
        msg = State()
        msg.header.stamp = rospy.Time.now()
        msg.position = error
        self.publishers[name].publish(msg)
        return True

    def loop(self):
        loop_timer = rospy.Rate(1000)
        while(not rospy.is_shutdown()):
            for name in self.filter_names:
                self.publish(name)
            loop_timer.sleep()
        return

    def __init__(self):
        # Node initialization
        rospy.init_node('sim_error_node')

        # Inner vars initialization
        self.positions = {
            "uncorrected"         : None,
            "velocity_correction" : None,
            "landmark_correction" : None,
            "complete_correction" : None, 
            "groundtruth" : None     
        }

        self.groundtruth_position = None
        self.complete_position = None
        self.velocity_position = None

        self.filter_names = [
            "uncorrected",
            "velocity_correction",
            "landmark_correction",
            "complete_correction"
        ]

        # Publishers
        self.publishers = {}
        for name in self.filter_names:
            self.publishers[name] = rospy.Publisher("/vehicle/state/error/state_{}".format(name), State,queue_size=1)

        # Subscription topics
        rospy.Subscriber("vehicle/state/filter/state_uncorrected", State, self.uncorrectedCallback)
        rospy.Subscriber("vehicle/state/filter/state_velocity_correction", State, self.velocityCallback)
        rospy.Subscriber("vehicle/state/filter/state_landmark_correction", State, self.landmarkCallback)
        rospy.Subscriber("vehicle/state/filter/state_complete_correction", State, self.completeCallback)
        rospy.Subscriber("vehicle/state/groundtruth/position", PointStamped, self.groundtruthCallback)
        
        # Loop
        self.loop()
        return

if __name__ == "__main__":
    ErrorNode()