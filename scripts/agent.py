#!/usr/bin/env python

"""
Listens for packets from the Omni node. Calculates its velocity based on the
information in the packets.
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from project_polygon.msg import Packet
import tf
import numpy as np
import helper_funcs as hp

# assume that we are receiving an info packet from Omni
# info packet has: R, centroid, k_a, k_b, k_c, location,
#   number of bots within region, other bot locations

class Agent:
    def __init__(self):
        rospy.init_node('Agent') # TODO: remember to change name for multiple bots

        self.command = Twist()
        self.last_stamp = rospy.Time.now()
        self.xy_vel = np.array([0.0, 0.0])

        self.x = None
        self.y = None
        self.yaw = None

        self.stamp = None
        self.centroid = None
        self.R = None
        self.k_a = None
        self.k_b = None
        self.k_c = None
        self.bot_qty = None
        self.bot_pos = None

        rospy.Subscriber('/packet', Packet, self.assign_data) # remember to change names for multiple bots
        rospy.Subscriber('/odom', Odometry, self.assign_odom)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

    def assign_odom(self, msg):
        """
        Save x, y, and yaw from odometry data
        """
        pose = msg.pose.pose
        self.x, self.y, self.yaw = hp.convert_pose_to_xy_and_theta(pose)
        print self.yaw

    def assign_data(self, data):
        """
        Unpack data packet and assign to self.attributes.
        """
        self.stamp = data.stamp
        self.centroid = data.centroid
        self.R = data.R
        self.k_a = data.k_a
        self.k_b = data.k_b
        self.k_c = data.k_c
        self.bot_qty = data.n  # number of robots it knows about (including self)
        self.bot_pos = data.others  # list of positions (excluding self)

    def calc_vels(self):
        """
        Calculate velocities - resultant, linear, angular - from data packet attributes.
        """
        dci = hp.euclid_dist((self.x, self.y), self.centroid)    # dist betw Bot and centroid

        # x- and y-accel of Bot to centroid
        acc_xci = -self.k_c * (dci - self.R) * (1/dci) * (self.x - self.centroid[0])
        acc_yci = -self.k_c * (dci - self.R) * (1/dci) * (self.y - self.centroid[1])

        # define x- and y-accel betw Bot and other bots
        acc_xij = 0.0
        acc_yij = 0.0

        # calculate L based on how many bots it knows about
        L = 2 * self.R * math.sin(math.pi/self.bot_qty)

        for j in range(len(self.bot_qty)):
            dij = euclid_dist((self.x, self.y), self.bot_pos[j])    # dist betw Bot and bot 

            # calc x- and y-accel betw Bot and bot j
            if dij <= L:
                acc_xij += -self.k_a * (dij - L) * (1/dij) * (self.x - self.bot_pos[j][0])
                acc_yij += -self.k_a * (dij - L) * (1/dij) * (self.y - self.bot_pos[j][1])

        # update accel
        acc = (acc_xci + acc_xij - k_b*self.xy_vel[0], acc_yci + acc_yij - self.k_b*self.xy_vel[1])

        # update vel using time since last update as timestep
        tau = (self.stamp - self.last_stamp).to_secs()
        self.last_stamp = self.stamp
        self.xy_vel = self.xy_vel + (acc * tau)    # new velocity = old velocity + (acceleration * timestep)

        self.xy_vel[self.xy_vel > 0.5] = 0.5  # set neato vel upper bound

    def move_bot(self):
        """
        Move the robot using velocities from calc_vels()
        """
        # do nothing if no packet has been received yet
        if self.stamp is None:
            return

        self.calc_vels()
        res_vel = complex(*self.xy_vel) # using complex number to represent vector
        angle = np.angle(res_vel)
        magnitude = abs(res_vel)

        angle_diff = hp.angle_diff(self.yaw, angle)    #for now, assume current angle is consistant with world
        self.command.linear.x = magnitude
        self.command.angular.z = angle_diff
        self.pub.publish(self.command)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.move_bot()
            r.sleep()

if __name__ == '__main__':
    node = Agent()
    node.run()
