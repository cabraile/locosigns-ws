#!/usr/bin/env python
import rospy 
import numpy
import rospkg
import tf
from gazebo_msgs.srv import SetModelState
from locosigns_msg.msg import Scalar, Pose
from prius_msgs.msg import Control
from gazebo_msgs.msg import ModelState 

class Controller():

    def __init__(self):
        rospy.init_node("sim_control_node")
        # Load args
        self.iter = 0.0
        self.x = rospy.get_param("~x", 0.0)             # meters
        self.y = rospy.get_param("~y", -1.5)            # meters
        self.z = rospy.get_param("~z", 0.0)             # meters
        self.heading = rospy.get_param("~heading", 0.0) # rads
        self.direction = rospy.get_param("direction", 1)# 1=forward, -1=backwards
        # Init args
        self.update_rate = 100.0
        self.steering = 0 # rads
        if(self.direction < 0):
            self.heading = numpy.pi -self.heading

        self.heading_direction = 1.0

        # Ackermann Steering - based on https://www.xarg.org/book/kinematics/ackerman-steering/
        self.body_wheel_base_length = 2.86012   # Specific to the prius model given (meters)
        self.body_track_width = 1.586           # Track width (meters)
        
        # Publishers
        self.control_pub = rospy.Publisher('prius', Control, queue_size=1)

        # Assures the vehicle did not move by any of the external forces of the simulation
        self.placeRobotOnMap()
        return

    # COMUNICATION
    # ========================

    def placeRobotOnMap(self):
        """
        Replace the default prius control module. Ignores the physics of the simulation.
        """
        # Gazebo
        state_msg = ModelState()
        state_msg.model_name = 'prius'
        state_msg.pose.position.x = self.x
        state_msg.pose.position.y = self.y
        state_msg.pose.position.z = self.z
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, self.heading)
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return
        return 

    def sendCtrlCmdsToGazebo(self):
        """
        Send control commands to Gazebo, which updates the robot state according to its inertial model.
        """
        command_msg = Control()
        command_msg.header.stamp = rospy.get_rostime()
        command_msg.throttle = 1.0
        command_msg.brake = 0.0
        command_msg.shift_gears = Control.FORWARD
        # Normalize steer angle to [-1, 1]
        steer = self.steering * 6.0 / numpy.pi
        command_msg.steer = steer
        self.control_pub.publish(command_msg)
        return

    # ========================

    # SIMULATION
    # ========================

    def loop(self):
        loop_timer = rospy.Rate(self.update_rate)
        while(not rospy.is_shutdown()):
            self.sendCtrlCmdsToGazebo()
            self.iter+=1
            loop_timer.sleep()
        return
    
    # ========================

if __name__ == '__main__':
    Controller().loop()