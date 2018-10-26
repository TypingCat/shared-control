#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import random
import math

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int32

from shared_control.srv import Nearest, Node


class EVALUATOR:
    """움직임을 평가한다"""
    def __init__(self):
        self.map = MapMetaData()
        self.state = -1                         # 상태: -1=초기화, 0=대기, 1=시작, 2=평가
        self.move = 0
        self.pose = Pose()
        self.dst_point = Point()                                        # 목적지 위치
        self.dst_margin = rospy.get_param('~destination_margin', 0.5)   # 목적지 반경
        self.dst_x_min = rospy.get_param('~destination_spawn_x_min', -5.0)
        self.dst_x_max = rospy.get_param('~destination_spawn_x_max', 5.0)
        self.dst_y_min = rospy.get_param('~destination_spawn_y_min', -5.0)
        self.dst_y_max = rospy.get_param('~destination_spawn_y_max', 5.0)
        self.time_start = 0.0

        rospy.wait_for_service('gvg/nearest')   # 서비스 초기화를 기다린다.
        rospy.wait_for_service('gvg/node')

        self.get_nearest = rospy.ServiceProxy('gvg/nearest', Nearest)
        self.get_node = rospy.ServiceProxy('gvg/node', Node)

        rospy.Subscriber('map', OccupancyGrid, self.load_map)
        rospy.Subscriber('robot/state', Int32, self.update_state)
        rospy.Subscriber('robot/pose', Pose, self.update_pose)

        self.publisher_dst = rospy.Publisher('interface/destination', Point, queue_size=1)

        rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.evaluation)

    def load_map(self, data):
        """지도 좌표를 획득한다"""
        self.map = data.info
        self.state = 0

    def evaluation(self, event):
        """주행을 평가한다"""
        if self.state == 0:                                 # 초기화중일 경우,
            try:                                            # 대기를 확인하고 다음으로 넘어간다.
                self.dst_point = self.select_destination().point
                if self.move == 0:
                    rospy.loginfo("목적지: [%f, %f]"%(self.dst_point.x, self.dst_point.y))
                    self.state = 1
            except: pass

        elif self.state == 1:                               # 대기상태일 경우,
            if self.move == 1:                              # 로봇이 움직인다면,
                self.time_start = rospy.get_time()          # 게임을 시작한다.
                self.state = 2

        elif self.state == 2:                               # 이동중일 경우,
            dist = math.sqrt((self.dst_point.x - self.pose.position.x)**2 +
                             (self.dst_point.y - self.pose.position.y)**2)
            if dist < self.dst_margin:                      # 목적지에 도달했는지 검사한다.
                rospy.loginfo('소요시간: %f초'%(rospy.get_time()-self.time_start))
                self.state = 0

        p = Point()                                         # 목적지를 발행한다.
        p.x = self.dst_point.x
        p.y = self.dst_point.y
        p.z = self.dst_margin
        self.publisher_dst.publish(p)

    def select_destination(self):
        """목적지를 무작위로 선택한다"""
        rand = Point()          # 지도로부터 무작위 위치를 선정한다.
        rand.x = (self.dst_x_max - self.dst_x_min)*random.random() + self.dst_x_min
        rand.y = (self.dst_y_max - self.dst_y_min)*random.random() + self.dst_y_min
        nearest = -1            # 가장 가까운 노드를 검색한다.
        while nearest == -1:
            nearest = self.get_nearest(rand).id
            rospy.sleep(0.1)

        return self.get_node(nearest)

    def update_state(self, data):
        """로봇의 상태를 갱신한다"""
        self.move = data.data

    def update_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.pose = data                                                # 현재 자세를 갱신한다.

        try:
            self.nearest[0] = self.get_nearest(self.pose.position).id   # 가장 가까운 노드를 갱신한다.
            p = self.get_node(self.nearest[0]).point
            self.nearest[1] = (p.x-self.pose.position.x)**2 + (p.y-self.pose.position.y)**2
        except: pass


if __name__ == '__main__':
    rospy.init_node('evaluator')
    evaluator = EVALUATOR()
    rospy.spin()
