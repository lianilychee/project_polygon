#!/usr/bin/env python
"""
Script for controlling the robots using the values from the gui
"""

import rospy
import time
from geometry_msgs.msg import Twist

from dynamic_reconfigure.server import Server
from project_polygon.cfg import DriverConfig

def callback(config, level):
    rospy.loginfo(config)

    # Do something with the config data here

    return config

if __name__ == '__main__':
    rospy.init_node('multirobot_gui', anonymous = True)

    srv = Server(DriverConfig, callback)
    rospy.spin()
