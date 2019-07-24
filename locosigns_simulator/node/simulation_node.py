#!/usr/bin/env python
import rospy 
import numpy
import rospkg
import tf
from locosigns_msg.msg import Scalar, Pose
from prius_msgs.msg import Control
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

class Controller():

    def __init__(self):
        rospy.init_node("simulation_node")
        # Load args
        self.iter = 0.0
        self.x = rospy.get_param("~x", 0.0)            # meters
        self.y = rospy.get_param("~y", -1.5)           # meters
        self.z = rospy.get_param("~z", 0.0)            # meters
        self.heading = rospy.get_param("~heading", 0.0)# rads
        self.velocity = rospy.get_param("~velocity", 15.0)  # meters per second
        self.direction = rospy.get_param("direction", 1)    # 1=forward, -1=backwards
        self.path_period = rospy.get_param("path_period")   # Period in seconds of the sinusoidal path cycle
        self.use_default_control = rospy.get_param("use_default_control", True) # Select the control of the robot: the default provided by the package `prius_control` (True) or the one implemented in this package (False)
        self.steering = 0
        if(self.direction < 0):
            self.heading = numpy.pi -self.heading

        self.heading_direction = 1.0
        # Ackermann Steering - based on https://www.xarg.org/book/kinematics/ackerman-steering/
        self.body_wheel_base_length = 2.86012   # Specific to the prius model given (meters)
        self.body_track_width = 1.586           # Track width (meters)

        # Init inner vars
        self.S = 0.0#self.x
        self.update_rate = 100 # hertz
        self.last_update = rospy.get_rostime()

        # Publishers
        self.state_position_pub = rospy.Publisher("/state/groundtruth/position", Scalar, queue_size=1)
        self.state_velocity_pub = rospy.Publisher("/state/groundtruth/velocity", Scalar, queue_size=1)
        self.pose_pub = rospy.Publisher("/state/groundtruth/pose", Pose, queue_size=1)
        self.control_pub = rospy.Publisher('prius', Control, queue_size=1)
        return

    # COMUNICATION
    # ========================

    def __placeOnMap(self):
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

    def __defaultControlCall(self):
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

    def __gazeboBroadcast(self):
        if not self.use_default_control:
            self.__placeOnMap()
        else:
            self.__defaultControlCall()
        return

    def __rosBroadcast(self):
        _pos_msg = Scalar()
        _pos_msg.header.stamp = rospy.get_rostime()
        _pos_msg.data = self.S
        self.state_position_pub.publish(_pos_msg)
        
        _vel_msg = Scalar()
        _vel_msg.header.stamp = rospy.get_rostime()
        _vel_msg.data = self.velocity
        self.state_velocity_pub.publish(_vel_msg)

        _pose_msg = Pose()
        _pose_msg.header.stamp = rospy.get_rostime()
        _pose_msg.pose.position.x = self.x
        _pose_msg.pose.position.y = self.y
        _pose_msg.pose.position.z = self.z

        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, self.heading)
        _pose_msg.pose.orientation.x = quaternion[0]
        _pose_msg.pose.orientation.y = quaternion[1]
        _pose_msg.pose.orientation.z = quaternion[2]
        _pose_msg.pose.orientation.w = quaternion[3]
        self.pose_pub.publish(_pose_msg)
        return
     
    def __broadcast(self):
        self.__gazeboBroadcast()
        self.__rosBroadcast()
        return

    # ========================

    # SIMULATION
    # ========================

    def __update(self):
        Delta_T = (1.0)/(1.0 * self.update_rate)
        # Every path_period seconds the direction of the steering changes (half of the sine cycle)
        self.steering = (numpy.pi / 6.0) * numpy.sin( (2.0 * numpy.pi * self.iter * Delta_T) / (self.path_period / 2.0) )
        heading_dot = numpy.tan(self.steering) * (self.velocity / self.body_wheel_base_length )
        self.heading += heading_dot * Delta_T # DOES NOT STEER DURING TIME INTERVALS
        x_dot = self.velocity * numpy.cos(self.heading)
        y_dot = self.velocity * numpy.sin(self.heading)
        self.x += x_dot * Delta_T
        self.y += y_dot * Delta_T
        self.z = 0.0
        arclength = self.velocity * Delta_T # FOR CONSTANT VELOCITY
        self.S += arclength
        return

    def loop(self):
        loop_timer = rospy.Rate(self.update_rate)
        while(not rospy.is_shutdown()):
            self.__update()
            self.__broadcast()
            self.iter+=1
            loop_timer.sleep()
        return
    
    # ========================

if __name__ == '__main__':
    Controller().loop()