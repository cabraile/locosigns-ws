#!/usr/bin/env python
import rospy 
import numpy
import rospkg 
from std_msgs.msg import Header
from locosigns_msg.msg import Scalar
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

class Controller():

    def __init__(self):
        rospy.init_node("simulation_node")
        # Load args
        self.x = rospy.get_param("~x", 0.0) # meters
        self.y = rospy.get_param("~y", -1.5) # meters
        self.z = rospy.get_param("~z", 0.0) # meters
        self.heading = rospy.get_param("~heading", 0.0) # rads
        self.velocity = rospy.get_param("~velocity", 15.0) # meters per second
        self.direction = rospy.get_param("direction", 1) # 1=forward, -1=backwards
        
        # Init inner vars
        self.S = self.x
        self.update_rate = 100 # hertz

        # Publishers
        self.state_position_pub = rospy.Publisher("/state/groundtruth/position", Scalar, queue_size=1)
        self.state_velocity_pub = rospy.Publisher("/state/groundtruth/velocity", Scalar, queue_size=1)
        return

    # COMUNICATION
    # ========================

    def __gazeboBroadcast(self):
        # Gazebo
        state_msg = ModelState()
        state_msg.model_name = 'prius'
        state_msg.pose.position.x = self.S
        state_msg.pose.position.y = -1.5
        state_msg.pose.position.z = 0.0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        if(self.direction == 1):
            state_msg.pose.orientation.z = self.heading
        else:
            state_msg.pose.orientation.z = numpy.pi - self.heading
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
        _pos_msg = Scalar()
        _vel_msg = Scalar()
        _pos_msg.header.stamp = rospy.get_rostime()
        _vel_msg.header.stamp = rospy.get_rostime()
        _pos_msg.data = self.S
        _vel_msg.data = self.velocity
        self.state_position_pub.publish(_pos_msg)
        self.state_velocity_pub.publish(_vel_msg)
        return
     
    def __broadcast(self):
        self.__gazeboBroadcast()
        self.__rosBroadcast()
        return

    # ========================

    # SIMULATION
    # ========================

    def __update(self):
        self.S += self.velocity * (1.0/(self.update_rate * 1.0) ) * (1.0 * self.direction)
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
    Controller().loop()