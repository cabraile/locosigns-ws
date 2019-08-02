#!/usr/bin/env python
import rospy
from numpy import *

import locosigns_msg.msg
import nav_msgs.msg

class Velocimeter():

    def callback(self, msg):
        # Message
        time = msg.header.stamp
        position = msg.pose.pose.position

        # Initial condition
        if(self.position is None):
            self.position = position
            self.time = time
            return

        # Measure
        time_offset = float((time - self.time).to_sec())
        distance = sqrt(
            (position.x - self.position.x) ** 2.0 +
            (position.y - self.position.y) ** 2.0 +
            (position.z - self.position.z) ** 2.0
        )
        velocity = distance/time_offset
        # Noise
        noise = ( 0.1 * velocity ) * (2.0 * random.rand() - 1.0)
        velocity_n = velocity + noise

        # Publish messages
        true_msg = locosigns_msg.msg.Scalar()
        true_msg.header.stamp = rospy.Time.now()
        true_msg.data = velocity
        self.true_publisher.publish(true_msg)
        noisy_msg = locosigns_msg.msg.Scalar()
        noisy_msg.header.stamp = rospy.Time.now()
        noisy_msg.data = velocity_n
        self.noisy_publisher.publish(noisy_msg)

        # Update
        self.position = position
        self.time = time
        return

    def __init__(self):
        # Node initialization
        rospy.init_node('sim_velocimeter_node')
        # Inner vars initialization
        self.position = None
        # Publishers
        self.true_publisher = rospy.Publisher("/sim_sensor/velocity_groundtruth", locosigns_msg.msg.Scalar,queue_size=1)
        self.noisy_publisher = rospy.Publisher("/sim_sensor/velocity", locosigns_msg.msg.Scalar,queue_size=1)
        # Subscription topics
        rospy.Subscriber("/base_pose_ground_truth", nav_msgs.msg.Odometry, self.callback)
        # Loop
        rospy.spin()
        return

if __name__ == "__main__":
    Velocimeter()