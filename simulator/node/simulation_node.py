#!/usr/bin/env python

import rospy 
import rospkg 
from std_msgs.msg import Float64, Float64MultiArray
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

class Controller():

    def __init__(self):
        self.S = 0.0
        self.update_rate = 10 # hertz
        self.velocity = 15.0 # meters per second
        self.state_velocity_pub = rospy.Publisher("/state/groudtruth/velocity", Float64)
        self.state_position_pub = rospy.Publisher("/state/groudtruth/position", Float64)
        return

    # COMUNICATION
    # ========================

    def __gazeboBroadcast(self):
        # Gazebo
        state_msg = ModelState()
        state_msg.model_name = 'prius'
        state_msg.pose.position.x = self.S
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return
        return

    def __rosBroadcast(self):
        curr_time = rospy.get_rostime()
        self.state_position_pub.publish(self.S)
        self.state_velocity_pub.publish(self.velocity)
        return
     
    def __broadcast(self):
        self.__gazeboBroadcast()
        self.__rosBroadcast()
        return

    # ========================

    # SIMULATION
    # ========================

    def __update(self):
        self.S += self.velocity * (1.0/(self.update_rate * 1.0) )
        return

    def loop(self):
        loop_timer = rospy.Rate(self.update_rate)
        while(not rospy.is_shutdown()):
            self.__update()
            self.__broadcast()
            loop_timer.sleep()
        return
    
    # ========================

if __name__ == '__main__':
    rospy.init_node('simulation_node')
    Controller().loop()