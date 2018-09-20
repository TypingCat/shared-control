#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy

import termios
import sys
import select
import tty

from std_msgs.msg import Int32
from shared_control.srv import MotorImagery


class FAKE_BCI:
    """BCI를 키보드로 대체한다"""
    def __init__(self):
        self.key = ''
        self.key_setting = termios.tcgetattr(sys.stdin)
        self.key_watcher = rospy.Timer(rospy.Duration(0.1), self.spin)

        self.publisher = rospy.Publisher('bci/eyeblink', Int32, queue_size=1)

        self.binary_question = []
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
                self.publisher.publish(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.key_setting)

    def motorimagery(self, request):
        """Motor imagery를 대신하여 binary question에 답변한다"""
        rospy.loginfo('[a:%d, d:%d] 어디로 이동할까요?'%request.ids)

        answer = -1
        while answer == -1:             # 답변이 들어올 때까지 키를 확인한다.
            if self.key == 'a':         # a가 들어오면 첫번째 값을 돌려준다.
                answer = request.ids[0]
            elif self.key == 'd':       # d가 들어오면 두번째 값을 돌려준다.
                answer = request.ids[1]

            rospy.sleep(0.1)

        return {'id': answer}


if __name__ == '__main__':
    rospy.init_node('fake_bci')
    fake_bci = FAKE_BCI()
    rospy.spin()
