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
import helper_funcs as hp
from copy import deepcopy


class Omni:
    def __init__(self, n):
        """
        Omni init class sets the variables, gets the robots' position, and publishes to 
        packet for instances of Agent to subscribe to. 
        """

        rospy.init_node('omniscient')

        #set all constants
        self.centroid = (5,0)
        self.k_a = 0.8
        self.k_b = 0.2
        self.k_c = 0.8
        self.R = 2
        self.scan_threshold=2 #the maximum scan radius that each agent can scan

        self.bot_qty = 6  #number of robots existing total
        self.bot_pos = np.zeros((self.bot_qty,3))  #create empty numpy array size of num of robots. 

        #subscribe to the robot to get its position in world coordinate frame
        self.pos_sub=rospy.Subscriber('/robot1/odom',Odometry,self.get_pos())


    def get_pos(self,msg):
        """
        call back function to get the position of robots to deploy to the agents.
        """

        self.bot_pos=msg.pose.pose
        #use helper function to retuurn a list of positions of robots that are within
        #R should publish this information each time with the robot's callback_args
        #where to publish??
        self.bots_within_R=self.neighbor_bots(self.bot_pos,callback_args)

    #might need converstion to (x,y) tuple in this function?
    def neighbor_bots(bots_pos,args):
        """
        neighbor_bots takes in the bot's callback args and position of other robotss
        Then it returns list of position of all other rots with in radius R """
        copy_bots_pos=[deepcopy(i) for i in bots_pos] #copy it before the bot_pos gets updated,avoiding threading issue
        neighbors=[]
        for i in len(bots_pos): #assume args is just number for now
            if (i != args) and (hp.euclid_dist(copy_bots_pos[args], copy_bots_pos[i]) <= scan_threshold):
                neighbors.append(copy_bots_pos[i])

        return neighbors

            

    def deploy(self):
        """
        deploys information to all agent nodes
        """





# subscribe to location of all bots

# reconcile agent odoms to world coordinate systems

# publish location of neighbor agents