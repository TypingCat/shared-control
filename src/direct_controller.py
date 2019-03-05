#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import termios
import sys
import select
import tty
import copy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DirectController:
    """키보드로 이동로봇을 직접 제어한다"""
    def __init__(self):
        self.robot_vel_lin = rospy.get_param('~robot_vel_lin', 0.26)

        self.robot_yaw = 0.
        rospy.Subscriber('odom', Odometry, self.update_robot_yaw)

        self.publisher_cmd_vec = rospy.Publisher('cmd_vec', Twist, queue_size=1)

        self.key_setting = termios.tcgetattr(sys.stdin)
        self.key_watcher = rospy.Timer(rospy.Duration(rospy.get_param('~freq', 0.1)), self.spin)

    def update_robot_yaw(self, data):
        """로봇의 자세를 갱신한다"""
        (_, _, self.robot_yaw) = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,
                                                  data.pose.pose.orientation.y,
                                                  data.pose.pose.orientation.z,
                                                  data.pose.pose.orientation.w,])

    def get_key(self):
        """키보드 입력을 획득한다"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.key_setting)
        return key

    def spin(self, event):
        """키보드 입력을 BCI 프로토콜로 변환한다"""
        cmd_vec = Twist()
        key = self.get_key()
        if key == '\x03':   # ctrl+c
            self.key_watcher.shutdown()
        elif key == 'a':    # left
            cmd_vec.linear.x = self.robot_vel_lin
            cmd_vec.angular.z = self.robot_yaw
        elif key == 'w':    # forward
            cmd_vec.linear.x = self.robot_vel_lin
            cmd_vec.angular.z = 0.
        elif key == 'd':    # right
            cmd_vec.linear.x = self.robot_vel_lin
            cmd_vec.angular.z = -self.robot_yaw
        elif key == 's':    # stop
            cmd_vec.linear.x = 0.
            cmd_vec.angular.z = 0.

        self.publisher_cmd_vec.publish(cmd_vec)


if __name__ == '__main__':
    rospy.init_node('direct_controller')
    dc = DirectController()
    rospy.spin()
