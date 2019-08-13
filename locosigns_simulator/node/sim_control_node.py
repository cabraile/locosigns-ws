#!/usr/bin/env python
import rospy 
import numpy
import rospkg
import tf
from gazebo_msgs.srv import SetModelState, GetModelState, SpawnModel
from locosigns_msgs.msg import Scalar, Pose
from prius_msgs.msg import Control
from gazebo_msgs.msg import ModelState 
from geometry_msgs.msg import Point, Quaternion, TwistStamped
from geometry_msgs.msg import Pose as GeometryPose

class Controller():

    def __init__(self):
        rospy.init_node("sim_control_node")
        # Load args
        self.spawn_y = rospy.get_param("spawn_y", 0.0)             # meters
        self.direction = rospy.get_param("direction", 1)# 1=forward, -1=backwards
        self.target_velocity = rospy.get_param("target_velocity")

        # Init inner vars
        self.steering = 0.0
        curr_time = rospy.Time.now().to_sec()
        self.velocity = 0.0
        self.lin_position = 0.0
        self.R = 4000.0/numpy.pi
        self.spawn_x = self.y(self.spawn_y)
        self.spawn_z = 0.0
        
        self.x = self.spawn_x
        self.y = self.spawn_y
        self.z = self.spawn_z
        self.heading = self.theta(self.y)
        # Init args
        self.update_rate = 100.0
        if(self.direction < 0):
            self.heading = numpy.pi -self.heading


        # Ackermann Steering - based on https://www.xarg.org/book/kinematics/ackerman-steering/
        self.body_wheel_base_length = 2.86012   # Specific to the prius model given (meters)
        self.body_track_width = 1.586           # Track width (meters)

        # Span targets
        self.model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.curr_dummy_idx = 0 # Follows dummies on the road
        self.spanTargetPositionDummies()

        # Publishers
        self.control_pub = rospy.Publisher('prius', Control, queue_size=1)
        self.linear_position_pub = rospy.Publisher('/sim_sensors/linear_position', Scalar, queue_size=1)
        # Subscribers
        rospy.Subscriber("/sim_sensors/speedometer_groundtruth", TwistStamped, self.velocityCallback)

        # Assures the vehicle did not move by any of the external forces of the simulation
        #self.placeDummiesOnMap()
        self.placeRobotOnMap()
        return



    # GENERAL EQUATIONS
    # =========================================================================

    def y(self, x):
        if(x <= 2.0 * self.R):
            return numpy.sqrt((8000.0/numpy.pi) * x - numpy.square(x) )
        else:
            return -numpy.sqrt( (4000.0/numpy.pi)**2 - numpy.square(x - (12000.0 / numpy.pi )) )
        return

    def theta(self, x):
        return numpy.arcsin(x / self.R)

    # =========================================================================
    # SIMULATION
    # =======================

    def spanTargetPositionDummies(self):
        self.position_dummies = []
        R = self.R
        N = 1000.0
        skip = 5
        # Generate both semi-circles
        X_sc_1 = numpy.linspace(self.spawn_y, 2.0 * R, N/skip, endpoint=True)
        Y_sc_1 = numpy.sqrt( (8000.0/numpy.pi) * X_sc_1 - numpy.square(X_sc_1) ) # Y values for the first semi-circle
        X_sc_2 = numpy.linspace(2.0 * R, 4.0 * R, N/skip, endpoint=True)
        Y_sc_2 = -numpy.sqrt( (4000.0/numpy.pi)**2 - numpy.square(X_sc_2 - (12000.0 / numpy.pi )) ) # Y values for the second semi-circle
        # Append points sequentially - important
        for idx in range(1,int(N/skip)):
            self.position_dummies.append( numpy.array([Y_sc_1[idx], X_sc_1[idx]]) )
        for idx in range(0,int(N/skip)):
            self.position_dummies.append( numpy.array([Y_sc_2[idx], X_sc_2[idx]]) )
        return

    def placeDummiesOnMap(self):
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        with open("/home/braile/.gazebo/models/construction_cone/model.sdf", "r") as f:
            model_xml = f.read()
        #for idx in range(30):
        for idx in range(len(self.position_dummies)):
            dummy = self.position_dummies[idx]
            name = "dummy_{}".format(idx)
            rospy.loginfo("[sim_control_node.py] Spawning model: {}".format( name))
            pose = GeometryPose(Point(x=dummy[0], y=dummy[1], z=0.0),  Quaternion() )
            spawn_model(name, model_xml, "", pose, "world")
        return 

    def placeRobotOnMap(self):
        """
        Replace the default prius control module. Ignores the physics of the simulation.
        """
        # Gazebo
        state_msg = ModelState()
        state_msg.model_name = 'prius'
        state_msg.pose.position.x = self.spawn_x
        state_msg.pose.position.y = self.spawn_y
        state_msg.pose.position.z = self.spawn_z
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

    def fetchCurrentDummy(self):
        if(self.curr_dummy_idx >= len(self.position_dummies)):
            return
        # Choose the closest dummy
        dummy = self.position_dummies[self.curr_dummy_idx]
        dist = numpy.sqrt((self.x - dummy[0] ) ** 2.0 + (self.y - dummy[1]) ** 2.0)
        if(dist < 0.7):
            self.curr_dummy_idx += 1
            dummy = self.position_dummies[self.curr_dummy_idx]
        return 

    # =======================

    # CONTROL
    # =======================

    def getTargetThrottle(self):
        if(self.curr_dummy_idx >= len(self.position_dummies)):
            self.target_velocity = 0.0
        velocity_error = self.target_velocity - self.velocity
        Delta_T = 1.0 / self.update_rate
        p_term = 0.8 * velocity_error
        i_term = 0.2 * velocity_error * Delta_T
        d_term = 0.1 * velocity_error * self.update_rate
        throttle =  p_term + i_term
        self.throttle = min(1., max(0., throttle) )
        return

    def getTargetSteering(self):
        # Choose the closest dummy
        dummy = self.position_dummies[self.curr_dummy_idx]

        # Get heading
        delta_x = (dummy[0] - self.x )
        delta_y = (dummy[1] - self.y)
        target_angle = numpy.arctan2(delta_y, delta_x)

        # The robot needs to face the landmark, however it depends on the steering
        angle_error = target_angle - self.heading
        #print(angle_error)
        target_steering = numpy.arctan(self.update_rate * angle_error * self.body_wheel_base_length / (self.body_track_width) )
        steering_error = target_steering - self.steering
        #steering_error = angle_error
        Delta_T = (1. / self.update_rate)
        # PID
        p_term = 0.05 * steering_error
        i_term = 0.8 * steering_error * Delta_T
        d_term = 0.9 * steering_error * self.update_rate
        steering = self.steering + p_term + i_term + d_term
        # Update
        steering = max(-0.5, min(0.5, steering) ) # MAX STEERING ANGLE IS 0.8727 RAD ~ 50deg
        self.steering = steering
        #print(self.steering)
        return

    # =======================

    # COMUNICATION
    # ========================
    
    def velocityCallback(self, msg):
        self.velocity = msg.twist.linear.x
        return

    def getRobotState(self):
        model_name = "prius"
        state = self.model_state(model_name, "")
        orient = state.pose.orientation
        orient = numpy.array([orient.x, orient.y, orient.z, orient.w])
        self.x = state.pose.position.x
        self.y = state.pose.position.y
        self.z = state.pose.position.z
        _,_,self.heading = tf.transformations.euler_from_quaternion(orient)
        curr_pos = numpy.array([self.x , self.y])
        curr_position_idx = self.curr_dummy_idx
        S = 0
        arclength = 0.
        for idx in range(1, curr_position_idx):
            dummy_curr = self.position_dummies[idx]
            dummy_prev = self.position_dummies[idx-1]
            arclength += numpy.linalg.norm(dummy_curr-dummy_prev)
        if(curr_position_idx > 0):
            arclength += numpy.linalg.norm( self.position_dummies[0] )
            arclength += numpy.linalg.norm(curr_pos - self.position_dummies[curr_position_idx-1])
        else:
            arclength = numpy.linalg.norm( curr_pos )
        S = arclength
        self.lin_position = S
        return

    def sendCtrlCmdsToGazebo(self):
        """
        Send control commands to Gazebo, which updates the robot state according to its inertial model.
        """
        command_msg = Control()
        command_msg.header.stamp = rospy.get_rostime()
        command_msg.throttle = self.throttle
        command_msg.brake = 0.0
        command_msg.shift_gears = Control.FORWARD
        steer = self.steering
        command_msg.steer = steer
        self.control_pub.publish(command_msg)

        position_msg = Scalar()
        position_msg.header.stamp = rospy.get_rostime()
        position_msg.data = self.lin_position
        self.linear_position_pub.publish(position_msg)
        return

    # ========================

    # SIMULATION
    # ========================

    def loop(self):
        loop_timer = rospy.Rate(self.update_rate)
        while(not rospy.is_shutdown()):
            self.fetchCurrentDummy()
            self.getRobotState()
            self.getTargetThrottle()
            self.getTargetSteering()
            self.sendCtrlCmdsToGazebo()
            loop_timer.sleep()
        return
    
    # ========================

if __name__ == '__main__':
    Controller().loop()