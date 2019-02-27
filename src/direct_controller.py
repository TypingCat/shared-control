#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import termios
import sys
import select
import tty

from std_msgs.msg import Int32


class DirectController:
    """키보드로 이동로봇을 직접 제어한다"""
    def __init__(self):
        self.publisher_command = rospy.Publisher('command', Int32, queue_size=1)

        self.key_setting = termios.tcgetattr(sys.stdin)
        self.key_watcher = rospy.Timer(rospy.Duration(rospy.get_param('~freq', 0.1)), self.spin)

    def get_key(self):
        """키보드 입력을 획득한다"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.key_setting)
        return key

    def spin(self, event):
        """키보드 입력을 BCI 프로토콜로 변환한다"""
        key = self.get_key()
        if key == '\x03':   # ctrl+c
            self.key_watcher.shutdown()
        elif key == 'a':    # left
            self.publisher_command.publish(0)
        elif key == 'w':    # forward
            self.publisher_command.publish(1)
        elif key == 'd':    # right
            self.publisher_command.publish(2)
        elif key == 's':    # stop
            self.publisher_command.publish(3)
        elif key == 'x':    # backward
            self.publisher_command.publish(4)


if __name__ == '__main__':
    rospy.init_node('direct_controller')
    dc = DirectController()
    rospy.spin()
