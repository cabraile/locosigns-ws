#!/usr/bin/env python
import rospy
from numpy import *
from locosigns_msgs.msg import Scalar, State
import nav_msgs.msg

class ErrorNode():

    def odometryCallback(self, msg):
        self.odometry = msg.position
        return

    def correctedOdometryCallback(self,msg):
        self.odometry_corrected = msg.position
        return

    def trueOdometryCallback(self, msg):
        self.groundtruth = msg.data
        return

    def loop(self):
        loop_timer = rospy.Rate(1000)
        while(not rospy.is_shutdown()):
            
            if(self.groundtruth is not None):

                if(self.odometry is not None):
                    odom_error_msg = Scalar()
                    odom_error_msg.header.stamp = rospy.get_rostime()
                    odom_error = abs(self.groundtruth - self.odometry)
                    odom_error_msg.data = odom_error
                    self.odom_error_publisher.publish(odom_error_msg)
                
                if(self.odometry_corrected is not None):
                    odom_corr_error_msg = Scalar()
                    odom_corr_error_msg.header.stamp = rospy.get_rostime()
                    odom_corr_error = abs(self.groundtruth - self.odometry_corrected)
                    odom_corr_error_msg.data = odom_corr_error
                    self.odom_corrected_error_publisher.publish(odom_corr_error_msg)

            loop_timer.sleep()
        return

    def __init__(self):
        # Node initialization
        rospy.init_node('sim_error_node')

        # Inner vars initialization
        self.groundtruth = None
        self.odometry = None
        self.odometry_corrected = None

        # Publishers
        self.odom_error_publisher = rospy.Publisher("/error/complete_correction", Scalar,queue_size=1)
        self.odom_corrected_error_publisher = rospy.Publisher("/error/velocity_correction", Scalar,queue_size=1)

        # Subscription topics
        rospy.Subscriber("/state/filter/state_complete_correction", State, self.odometryCallback)
        rospy.Subscriber("/state/filter/state_velocity_correction", State, self.correctedOdometryCallback)
        rospy.Subscriber("/sim_sensors/linear_position", Scalar, self.trueOdometryCallback)
        
        # Loop
        self.loop()

        return

if __name__ == "__main__":
    ErrorNode()