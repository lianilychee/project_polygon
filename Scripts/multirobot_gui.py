#!/usr/bin/env python
'''Using the cmd_vel topic tells the robot to move forward turn left then
repeat 4 times then finally stops'''
import rospy
import time
from geometry_msgs.msg import Twist

from dynamic_reconfigure.server import Server
from multirobot_driver.cfg import driverConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
          {str_param}, {bool_param}, {size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("multirobot_gui", anonymous = True)

    srv = Server(driverConfig, callback)
    rospy.spin()