#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import termios
import sys
import select
import tty

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray, Marker

from shared_control.srv import MotorImagery, Node


class FAKE_BCI:
    """BCI를 키보드로 대체한다"""
    def __init__(self):
        self.key = ''
        self.key_setting = termios.tcgetattr(sys.stdin)
        self.key_watcher = rospy.Timer(rospy.Duration(0.1), self.spin)
        self.pose = Pose()

        rospy.Subscriber('robot/pose', Pose, self.update_pose)

        self.eyeblink_publisher = rospy.Publisher('bci/eyeblink', Int32, queue_size=1)
        self.marker_publisher = rospy.Publisher('bci/marker', MarkerArray, queue_size=1)

        rospy.Service('bci/motorimagery', MotorImagery, self.motorimagery)

        rospy.wait_for_service('gvg/node')
        self.get_node = rospy.ServiceProxy('gvg/node', Node)

    def __del__(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.key_setting)

    def spin(self, event):
        """키보드 입력을 획득한다"""
        tty.setraw(sys.stdin.fileno())  # 키보드와 연결한다.

        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            self.key = sys.stdin.read(1)
            if self.key == '\x03':      # ctrl+c가 들어오면 키보드와의 연결을 종료한다.
                self.key_watcher.shutdown()
            elif self.key == 'w':       # Eye blink를 대신하여 trigger를 발행한다.
                self.eyeblink_publisher.publish(2)
        else:
            self.key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.key_setting)

    def motorimagery(self, request):
        """Motor imagery를 대신하여 binary question에 답변한다"""
        rospy.loginfo('[a:%d, d:%d] 어디로 이동할까요?'%request.ids)

        answer = -1
        key = 'fini'
        while (key != 'a')&(key != 'd'):    # 답변이 들어올 때까지 키를 확인한다.
            key = self.key

            p0 = self.pose.position         # 선택지를 마커로 출력한다.
            if request.ids[0] != -1:
                p0 = self.get_node(request.ids[0]).point
            p1 = self.pose.position
            if request.ids[1] != -1:
                p1 = self.get_node(request.ids[1]).point
            self.visualize_question(p0, p1)

            if self.key == 'a':             # a가 들어오면 첫번째 값을 돌려준다.
                answer = request.ids[0]
            elif self.key == 'd':           # d가 들어오면 두번째 값을 돌려준다.
                answer = request.ids[1]

            rospy.sleep(0.1)

        return {'id': answer}

    def visualize_question(self, p0, p1):
        q0 = Marker()                               # 마커 0을 생성한다.
        q0.header.stamp = rospy.Time.now()
        q0.header.frame_id = 'map'
        q0.id = 0
        q0.type = Marker.CYLINDER
        q0.action = Marker.ADD
        q0.scale.x = 0.2
        q0.scale.y = 0.2
        q0.scale.z = 0.2
        q0.pose.position = p0
        q0.color.r = 1.0
        q0.color.a = 0.7

        q1 = Marker()                               # 마커 1을 생성한다.
        q1.header.stamp = rospy.Time.now()
        q1.header.frame_id = 'map'
        q1.id = 1
        q1.type = Marker.CYLINDER
        q1.action = Marker.ADD
        q1.scale.x = 0.2
        q1.scale.y = 0.2
        q1.scale.z = 0.2
        q1.pose.position = p1
        q1.color.b = 1.0
        q1.color.a = 0.7

        self.marker_publisher.publish([q0, q1])     # 마커를 출력한다.

    def update_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.pose = data


if __name__ == '__main__':
    rospy.init_node('fake_bci')
    fake_bci = FAKE_BCI()
    rospy.spin()
