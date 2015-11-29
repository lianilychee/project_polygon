#!/usr/bin/env python

"""
Instantiates all agents, and knows all information of regarding all agents.  Reconcile agent base_links to world coordinate frame.  Sends packet information to each agent for individual path-planning.
"""

"""
bot 1 @ (0,0)
bot 2 @ (10, 1)
bot 3 @ (3, 3)
TODO: Going to just assume that this is a single robot for now...
"""

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Vector3
import tf
import numpy as np
import math
import agent
from project_polygon.msg import Packet

class Omni:
    # def __init__(self, n):
    def __init__(self):
        """
        Initialize node
        n = number of bots in existence
        """

        rospy.init_node('omniscient')

        rospy.Subscriber('/robot1/odom', Odometry, self.send_pkt)

        print 'init()'

        #put the position of robots in packet, make it in a numpy array.


        # publisher
        self.pub_pkt = rospy.Publisher('robot1/packet', Packet, queue_size=10)

        # packet values
        self.centroid = [2.0, 0.0]  # centroid location
        self.R = 1.0                # formation radius
        self.k_a = 0.08             # bot-bot constant
        self.k_b = 0.2              # damping constant
        self.k_c = 0.08             # bot-centroid constant
        self.n = 3                  # number of bots
        
        # PERHAPS THIS WILL BECOME A DICTIONARY
        self.others = [             # Poses of other bots
            Pose(position=Point(x=2+math.sqrt(2)/2, y=math.sqrt(2)/2)),
            Pose(position=Point(x=2+math.sqrt(2)/2, y=-math.sqrt(2)/2))
        ]

        # self.positions = # blank dictionary, to be filled later


    def id_neighbors(self):
        """
        for any given bot, identify positions of all neighbors within its sensing range
        """
        pass


    def send_pkt(self, msg):
        """
        define packet contents, then publish to appropriate topic
        """

        # print msg.pose.pose
        
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
    node = Omni()
    node.run()


# subscribe to location of all bots

# reconcile agent odoms to world coordinate systems

# publish location of neighbor agents