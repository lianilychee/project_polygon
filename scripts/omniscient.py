#!/usr/bin/env python

"""
Instantiates all agents, and knows all information of regarding all agents.  Reconcile agent base_links to world coordinate frame.  Sends packet information to each agent for individual path-planning.
"""

import rospyfrom sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Vector3
from sensor_msgs.msg import LaserScan, Image
import tf
import numpy as np
import math
import agent

class Omni:
    def __init__(self, n):
        """
        n = number of bots in existence
        """

        rospy.init_node('omniscient')

        # subscribe to all bots
        for i in range(n):
            print '/robot'+n+'/odom'
            self.Subscriber('/robot'+n+'/odom')


        #put the position of robots in packet, make it in a numpy array.



    def deploy(self):
        """
        deploys information to all agent nodes
        """




# subscribe to location of all bots

# reconcile agent odoms to world coordinate systems

# publish location of neighbor agents