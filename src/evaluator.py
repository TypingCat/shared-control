#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import random

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point
from std_msgs.msg import Int32

from shared_control.srv import Nearest, Node


class EVALUATOR:
    """움직임을 평가한다"""
    def __init__(self):
        self.map = MapMetaData()
        self.state = -1                         # 상태: -1=초기화, 0=대기, 1=평가
        self.move = [0, 0]

        rospy.wait_for_service('gvg/nearest')   # 서비스 초기화를 기다린다.
        rospy.wait_for_service('gvg/node')

        self.get_nearest = rospy.ServiceProxy('gvg/nearest', Nearest)
        self.get_node = rospy.ServiceProxy('gvg/node', Node)

        rospy.Subscriber('map', OccupancyGrid, self.load_map)
        rospy.Subscriber('robot/state', Int32, self.update_state)

        rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.evaluation)

    def load_map(self, data):
        """지도 좌표를 획득한다"""
        self.map = data.info
        self.state = 0

    def update_state(self, data):
        """로봇의 상태를 갱신한다"""
        self.move[1] = self.move[0]
        self.move[0] = data.data

    def evaluation(self, event):
        if self.state == 0:
            pass
        elif self.state == 1:
            pass



    #     dst = self.select_destination()
    #
    # def select_destination(self):
    #     """목적지를 무작위로 선택한다"""
    #     rand = Point()          # 지도로부터 무작위 위치를 선정한다.
    #     rand.x = random.randrange(0, self.map.width)*self.map.resolution + self.map.origin.position.x
    #     rand.y = random.randrange(0, self.map.height)*self.map.resolution + self.map.origin.position.y
    #
    #     nearest = -1            # 가장 가까운 노드를 검색한다.
    #     while nearest == -1:
    #         nearest = self.get_nearest(rand).id
    #         rospy.sleep(0.1)
    #
    #     return self.get_node(nearest)


if __name__ == '__main__':
    rospy.init_node('evaluator')
    evaluator = EVALUATOR()
    rospy.spin()
