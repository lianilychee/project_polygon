#!/usr/bin/env python

"""
Instantiates all agents, and knows all information of regarding all agents.  Reconcile agent base_links to world coordinate frame.  Sends packet information to each agent for individual path-planning.
bot 1 @ (0,0)
bot 2 @ (10, 1)
bot 3 @ (3, 3)
TODO: Going to just assume that this is a single robot for now...
"""
import rospy
import tf
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import math
import agent
import helper_funcs as hp
from copy import deepcopy
from project_polygon.msg import Packet


class Omni:
    # def __init__(self, n):
    def __init__(self):
        """
        Omni init class sets the variables, gets the robots' position, and publishes to packet for instances of Agent to subscribe to. 
        """

        rospy.init_node('omniscient')

        # # subscribe to all bots Will use it later when we figure out the multiple robot problem
        self.pub=[]
        for i in range(n):
            rospy.Subscriber('/robot{}/odom'.format(i), Odometry, self.get_pos, callback_args=i)

        #set all constants
        self.centroid = (5,0)
        self.k_a = 0.8
        self.k_b = 0.2
        self.k_c = 0.8
        self.R = 2

        self.scan_threshold=2 #the maximum scan radius that each agent can scan

        self.bot_pos = [None]*n  #create empty lint with a size of num of robots. 


    def get_pos(self,msg,callback_args):

        """
        call back function to get the position of robots to deploy to the agents.
        """
        self.bot_pos[callback_args]=msg.pose.pose


    #might need converstion to (x,y) tuple in this function?
    def neighbor_bots(bots_pos,args):
        """
        neighbor_bots takes in the bot's callback args and position of other robotss
        Then it returns list of position of all other rots with in radius R """
        copy_bots_pos=[deepcopy(i) for i in bots_pos] #copy it before the bot_pos gets updated,avoiding threading issue
        neighbors=[]

        # TODO: UNCOMMENT WHEN OTHER ARGS GET PUT IN
        # for i in len(bots_pos): #assume args is just number for now
        #     if (i != args) and (hp.euclid_dist(copy_bots_pos[args], copy_bots_pos[i]) <= scan_threshold):
        #         neighbors.append(copy_bots_pos[i])

        return neighbors


    def send_pkt(self, msg):
        """
        define packet contents, then publish to appropriate topic
        """

        # print msg.pose.pose
        #use helper function to retuurn a list of positions of robots that are within
        #R should publish this information each time with the robot's callback_args
        #where to publish??
        self.bots_within_R=self.neighbor_bots(self.bot_pos,callback_args)

        my_packet = Packet(
            centroid = self.centroid,
            R = self.R,
            k_a = self.k_a,
            k_b = self.k_b,
            k_c = self.k_c,
            n = self.n,
            others = self.others
        )


        my_packet.header.stamp = rospy.Time.now()
        self.pub_pkt.publish(my_packet)

    def run(self):
        # pass
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # self.send_pkt() # needs callback args to determine which topic to publish to
            r.sleep()


if __name__ == '__main__':
    node = Omni(4)
    node.run()


# reconcile agent odoms to world coordinate systems

# publish location of neighbor agents

