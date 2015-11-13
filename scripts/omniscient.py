#!/usr/bin/env python

"""
Listens to odom topics of all agents.  Reconcile agent odoms to world coordinate
system.  Publish location of neighbor agents if agents are within a certain 
distance of each other.
"""

import rospy
import tf
import numpy as np

# subscribe to location of all bots

# reconcile agent odoms to world coordinate systems

# publish location of neighbor agents