#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import tf
import termios, sys, select, tty

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

C_RED   = "\033[31m"
C_GREEN = "\033[32m"
C_YELLO = "\033[33m"
C_END   = "\033[0m"


class DirectControl:
    """이동로봇을 직접 제어한다"""
    def __init__(self):
        self.robot_vel_lin = rospy.get_param('~robot_vel_lin', 0.26)
        self.robot_vel_ang = rospy.get_param('~robot_vel_ang', 1.82)

        rospy.Subscriber('joy', Joy, self.joystick)

        self.publisher_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.key_setting = termios.tcgetattr(sys.stdin)
        self.key_watcher = rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.keyboard)
        print(C_GREEN + '\rDirect controller, 초기화 완료' + C_END)

    def get_key(self):
        """키보드 입력을 획득한다"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.key_setting)
        return key

    def keyboard(self, event):
        """키보드 입력을 제어명령으로 변환한다"""
        cmd_vel = Twist()
        key = self.get_key()
        if key == '\x03':   # ctrl+c
            self.key_watcher.shutdown()
        elif key == 'a':    # left
            cmd_vel.linear.x = 0.
            cmd_vel.angular.z = self.robot_vel_ang
        elif key == 'w':    # forward
            cmd_vel.linear.x = self.robot_vel_lin
            cmd_vel.angular.z = 0.
        elif key == 'd':    # right
            cmd_vel.linear.x = 0.
            cmd_vel.angular.z = -self.robot_vel_ang
        elif key == 's':    # stop
            cmd_vel.linear.x = 0.
            cmd_vel.angular.z = 0.
        elif key == 'x':    # backward
            cmd_vel.linear.x = -self.robot_vel_lin
            cmd_vel.angular.z = 0.
        else:
            return

        self.publisher_cmd_vel.publish(cmd_vel)

    def joystick(self, data):
        """조이스틱 입력을 제어명령으로 변환한다"""
        cmd_vel = Twist()
        cmd_vel.linear.x = self.robot_vel_lin*data.axes[1]
        cmd_vel.angular.z = self.robot_vel_ang*data.axes[0]

        self.publisher_cmd_vel.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('direct_controller')
    dc = DirectControl()
    rospy.spin()
