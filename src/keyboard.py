#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import termios
import sys
import select
import tty

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray, Marker

from shared_control.srv import MotorImagery, Node


class KEYBOARD:
    """BCI를 키보드로 대체한다"""
    def __init__(self):
        self.key = ''
        self.key_setting = termios.tcgetattr(sys.stdin)
        self.key_watcher = rospy.Timer(rospy.Duration(0.1), self.spin)
        self.pose = Pose()

        rospy.wait_for_service('gvg/node')
        self.get_node = rospy.ServiceProxy('gvg/node', Node)

        rospy.Subscriber('robot/pose', Pose, self.update_pose)

        self.publisher_eyeblink = rospy.Publisher('bci/eyeblink', Int32, queue_size=1)

        rospy.Service('bci/motorimagery', MotorImagery, self.motorimagery)

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
            elif self.key == 's':       # Eye blink를 대신하여 trigger를 발행한다.
                self.publisher_eyeblink.publish(2)
        else:
            self.key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.key_setting)

    def motorimagery(self, request):
        """Motor imagery를 대신하여 binary question에 답변한다"""
        answer = -1
        key = 'fini'
        while (key != 'a')&(key != 'd'):    # 답변이 들어올 때까지 키를 확인한다.
            key = self.key
            if self.key == 'a':             # a가 들어오면 첫번째 값을 돌려준다.
                answer = request.ids[0]
            elif self.key == 'd':           # d가 들어오면 두번째 값을 돌려준다.
                answer = request.ids[1]

            rospy.sleep(0.1)

        return {'id': answer}

    def update_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.pose = data


if __name__ == '__main__':
    rospy.init_node('keyboard')
    keyboard = KEYBOARD()
    rospy.spin()
