#!/usr/bin/env python

"""
Listens to odom topics of all agents.  Reconcile agent odoms to world coordinate
system.  Publish location of neighbor agents if agents are within a certain 
distance of each other.
"""

import rospyfrom sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Vector3
from sensor_msgs.msg import LaserScan, Image
import tf
import numpy as np

class Omni:
    def __init__(self):

        ### ROS INITIALIZATION ###
        rospy.init_node('omniscient')

        # what am i subscribing to?
        self.Subscriber('/robot1/odom')
        self.Subscriber('/robot1/bump')
        self.Subscriber('/robot2/odom')
        self.Subscriber('/robot2/bump')
        self.Subscriber('/robot3/odom')
        self.Subscriber('/robot3/bump')
        # what am I publiishing to?

    def deploy(self):
        """
        deploys information to all agent nodes
        """




# subscribe to location of all bots

# reconcile agent odoms to world coordinate systems

# publish location of neighbor agents