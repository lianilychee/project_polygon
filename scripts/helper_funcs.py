"""
Helper functions for project_polygon
"""

import tty
import select
import termios
import sys
import math
from tf.transformations import euler_from_quaternion


def get_key(settings):
    """
    Return pressed key
    """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def euclid_dist(pt1, pt2):
    """
    Given two positions, calculate the euclidean distance between them
    """
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)


def convert_pose_to_xy_and_theta(pose):
    """
    Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple
    """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])


def pose_euclid_dist(pose1, pose2):
    """
    Given two poses, calculate the euclidean distance between them
    """
    xy_theta1 = convert_pose_to_xy_and_theta(pose1)
    xy_theta2 = convert_pose_to_xy_and_theta(pose2)
    return euclid_dist(xy_theta1[:-1], xy_theta2[:-1])


def angle_normalize(z):
    """
    Convenience function to map an angle to the range [-pi,pi]
    """
    return math.atan2(math.sin(z), math.cos(z))


def angle_diff(a, b):
    """
    Calculates the difference between angle a and angle b (both should be in
    radians) the difference is always based on the closest rotation from
    angle a to angle b
    examples:
        angle_diff(.1,.2) -> -.1
        angle_diff(.1, 2*math.pi - .1) -> .2
        angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a - b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2
