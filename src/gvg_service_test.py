#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy

from geometry_msgs.msg import Point

from navi_map.srv import Nearest, Neighbors, Node


class GVG_SERVICE_TEST:
    def __init__(self):
        rospy.wait_for_service('gvg/nearest')               # 서비스들을 등록한다.
        self.get_nearest = rospy.ServiceProxy('gvg/nearest', Nearest)
        rospy.wait_for_service('gvg/neighbors')
        self.get_neighbors = rospy.ServiceProxy('gvg/neighbors', Neighbors)
        rospy.wait_for_service('gvg/node')
        self.get_node = rospy.ServiceProxy('gvg/node', Node)

        rospy.Timer(rospy.Duration(2), self.call_service)   # 일정 주기마다 서비스들을 사용한다.

    def call_service(self, event):
        """GVG 서비스를 사용한다"""
        p = Point()                     # 임의의 위치 p를 선정한다.
        p.x =  1.38
        p.y = -3.16

        n = self.get_nearest(p)         # p와 가장 가까운 노드 n의 id를 획득한다.
        rospy.loginfo(n.id)

        ns = self.get_neighbors(n.id)   # n의 이웃노드들 ns의 id를 획득한다.
        rospy.loginfo(ns.id)

        node = self.get_node(n.id)      # n의 위치를 획득한다.
        rospy.loginfo(node.point)


if __name__ == '__main__':
    rospy.init_node('gvg_service_test')
    gvg_service_test = GVG_SERVICE_TEST()
    rospy.spin()
