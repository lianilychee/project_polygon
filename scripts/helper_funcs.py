"""
Helper functions for project_polygon
"""

import math
import numpy as np

def euclid_dist(pt1, pt2):
    """
    given two positions, calculate the euclid_dist
    """
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)



    



