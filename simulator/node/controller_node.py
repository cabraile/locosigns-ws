#!/usr/bin/env python

import rospy
from prius_msgs.msg import Control

class Controller():

    def __init__(self):
        self.pub = rospy.Publisher('prius', Control, queue_size=1)
        self.loop()
        return

    def sendCommand(self, msg):
        command = Control()
        command.header.stamp = rospy.Time.now()
        if msg["throttle"] >= 0:
            command.throttle = msg["throttle"]
            command.brake = 0.0
        else:
            command.brake = msg["throttle"] * -1
            command.throttle = 0.0

        if msg["command"] == "FORWARD":
            command.shift_gears = Control.FORWARD
        elif msg == "NEUTRAL":
            command.shift_gears = Control.NEUTRAL
        elif msg == "REVERSE":
            command.shift_gears = Control.REVERSE
        else:
            command.shift_gears = Control.NO_COMMAND

        command.steer = msg["steer"]
        self.pub.publish(command)
        return

    def loop(self):
        loop_timer = rospy.Rate(10)
        while(not rospy.is_shutdown()):
            msg = {"throttle" : 1, "command" : "FORWARD", "steer" : 0 }
            self.sendCommand(msg)
            loop_timer.sleep()
        return

if __name__ == '__main__':
    rospy.init_node('controller')
    Controller()