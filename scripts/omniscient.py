#!/usr/bin/env python

"""
Instantiates all agents, and knows all information of regarding all agents.  Reconcile agent base_links to world coordinate frame.  Sends packet information to each agent for individual path-planning.
"""

import rospyfrom sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
import tf
import numpy as np
import math
import agent

class Omni:
    def __init__(self, n):
        """
        Omni init class sets the variables, gets the robots' position, and publishes to 
        packet for instances of Agent to subscribe to. 
        """

        rospy.init_node('omniscient')

        # # subscribe to all bots Will use it later when we figure out the multiple robot problem
        # for i in range(n):
        #     print '/robot'+n+'/odom'
        #     self.Subscriber('/robot'+n+'/odom')


        #set all constants
        self.centroid = (5,0)
        self.k_a = 0.8
        self.k_b = 0.2
        self.k_c = 0.8
        self.R = 2
        self.bot_qty = n  #number of robots existing total
        self.bot_pos = np.zeros((n,3))  #create empty numpy array size of num of robots. 

        #subscribe to the robot to get its position in world coordinate frame
        self.pos_sub=rospy.Subscriber('/robot1/odom',Odometry,self.get_pos())


    def get_pos(self,msg):
        """
        call back function to get the position of robots
        """

        self.bot_pos=msg.pose.pose



    def deploy(self):
        """
        deploys information to all agent nodes
        """





# subscribe to location of all bots

# reconcile agent odoms to world coordinate systems

# publish location of neighbor agents