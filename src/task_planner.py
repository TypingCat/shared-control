#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import numpy
import time
import networkx

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from navi_map.srv import Nearest, Neighbors, Node


class TASK_PLANNER:
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('task_planner')
    task_planner = TASK_PLANNER()
    rospy.spin()
