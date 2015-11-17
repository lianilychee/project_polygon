#!/usr/bin/env python

"""
Listens to odom topic.  Reconcile agent odoms to world coordinate
system.  Publish location of neighbor agents if agents are within a certain 
distance of each other.
"""

import rospyfrom sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Vector3
from sensor_msgs.msg import LaserScan, Image
import tf
import numpy as np

class Agent:
    def __init__(self):

        # what am I publiishing to?


    def two_closest(self):
        """
        
        """



# subscribe to location of all bots

# reconcile agent odoms to world coordinate systems

# publish location of neighbor agents