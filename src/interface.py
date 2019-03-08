#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import termios
import sys
import select
import tty

from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray, Marker

from shared_control.msg import MotorImageryCue, MotorImageryResult, EyeblinkResult
from shared_control.srv import MotorImagery, Node

C_RED   = "\033[31m"
C_GREEN = "\033[32m"
C_YELLO = "\033[33m"
C_END   = "\033[0m"


class Interface:
    """사용자 입력을 관리한다"""
    def __init__(self):
        print(C_YELLO + 'Interfacer, BCI 서비스 준비중...' + C_END)
        self.publisher_motorimagery_cue = rospy.Publisher('interf/motorimagery_cue', MotorImageryCue, queue_size=1)
        rospy.Subscriber('interf/motorimagery_result', MotorImageryResult, queue_size=1)
        rospy.Subscriber('interf/eyeblink', EyeblinkResult, queue_size=1)
        rospy.Service('interf/motorimagery', MotorImagery, self.motorimagery)
        print(C_YELLO + 'Interfacer, BCI 서비스 시작' + C_END)

        self.publisher_motorimagery_result = rospy.Publisher('interf/motorimagery_result', MotorImageryResult, queue_size=1)
        self.key_setting = termios.tcgetattr(sys.stdin)
        self.key_watcher = rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.keyboard)
        print(C_GREEN + 'Interfacer, 초기화 완료' + C_END)

    def get_key(self):
        """키보드 입력을 획득한다"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.key_setting)
        return key

    def keyboard(self, event):
        """키보드 입력을 직관적인 명령으로 변환한다"""
        pass
        # key = self.get_key()
        # if key == '\x03':   # ctrl+c
        #     self.key_watcher.shutdown()
        # elif key == 'a':    # left
        #     self.publisher_motorimagery_result.publish(1)
        # elif key == 'w':    # forward
        #     self.publisher_motorimagery_result.publish(2)
        # elif key == 'd':    # right
        #     self.publisher_motorimagery_result.publish(3)
        # elif key == 's':    # stop
        #     self.publisher_motorimagery_result.publish(4)
        # elif key == 'x':    # backward
        #     self.publisher_motorimagery_result.publish(5)
        # else:
        #     return

    def motorimagery(self, request):
        """Motorimagery를 위한 서비스를 관리한다"""

        # self.publisher_motorimagery_cue.publish(self.seq)


        # answer = -1
        # key = 'fini'
        # while (key != 'a')&(key != 'd'):    # 답변이 들어올 때까지 키를 확인한다.
        #     key = self.key
        #     if self.key == 'a':             # a가 들어오면 첫번째 값을 돌려준다.
        #         answer = request.ids[0]
        #     elif self.key == 'd':           # d가 들어오면 두번째 값을 돌려준다.
        #         answer = request.ids[1]
        #
        #     rospy.sleep(0.1)

        return {'dir': 0}


if __name__ == '__main__':
    rospy.init_node('interfacer')
    i = Interface()
    rospy.spin()
