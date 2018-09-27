#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32

from navi_map.srv import Nearest, Neighbors, Node


class TASK_PLANNER:
    """로봇의 행동방침을 결정한다"""
    def __init__(self):
        self.state = 0                          # 로봇의 상태: 0=정지, 1=이동중
        self.pose = Pose()                      # 로봇의 자세

        rospy.Subscriber('bci/eyeblink', Int32, self.percussion)
        rospy.Subscriber('robot/state', Int32, self.update_state)
        rospy.Subscriber('robot/pose', Pose, self.update_pose)

        rospy.wait_for_service('gvg/nearest')   # 서비스들을 등록한다.
        rospy.wait_for_service('gvg/neighbors')
        rospy.wait_for_service('gvg/node')
        self.get_nearest = rospy.ServiceProxy('gvg/nearest', Nearest)
        self.get_neighbors = rospy.ServiceProxy('gvg/neighbors', Neighbors)
        self.get_node = rospy.ServiceProxy('gvg/node', Node)

        self.target_publisher = rospy.Publisher('robot/target', Pose, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.spin)

    def spin(self, event):
        pass

    def percussion(self, data):
        pass

    def update_state(self, data):
        """로봇의 상태를 갱신한다"""
        self.state = data

    def update_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.pose = data


if __name__ == '__main__':
    rospy.init_node('task_planner')
    task_planner = TASK_PLANNER()
    rospy.spin()
