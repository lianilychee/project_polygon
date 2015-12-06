#!/usr/bin/env python
"""
Test of the custom Packet message
"""

import math
import rospy
from geometry_msgs.msg import Pose, Point
from project_polygon.msg import Packet


class PacketSender(object):
    """
    Node for sending Packets
    """

    def __init__(self):
        """
        Initialize node
        """
        rospy.init_node('packet_sender')

        # Publisher
        self.pub = rospy.Publisher('robot1/packet', Packet, queue_size=10)

        # Values
        self.centroid = [2.0, 0.0]  # centroid location
        self.R = 1.0                # formation radius
        self.k_a = 0.08             # bot-bot constant
        self.k_b = 0.2              # damping constant
        self.k_c = 0.08             # bot-centroid constant
        self.n = 3                  # number of bots
        self.others = [             # Poses of other bots
            Pose(position=Point(x=2+math.sqrt(2)/2, y=math.sqrt(2)/2)),
            Pose(position=Point(x=2+math.sqrt(2)/2, y=-math.sqrt(2)/2))
        ]

    def run(self):
        """
        The main run loop, publish Packet messages
        """
        r = rospy.Rate(5)
        my_packet = Packet(
            centroid = self.centroid,
            R = self.R,
            k_a = self.k_a,
            k_b = self.k_b,
            k_c = self.k_c,
            n = self.n,
            others = self.others
        )
        while not rospy.is_shutdown():
            my_packet.header.stamp = rospy.Time.now()
            self.pub.publish(my_packet)
            r.sleep()

if __name__ == '__main__':
    node = PacketSender()
    node.run()
