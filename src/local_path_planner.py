#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import copy

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


class LocalPathPlanner:
    """지역경로를 계획한다"""
    def __init__(self):
        self.max_vel_lin = rospy.get_param('~max_vel_lin', 0.26)
        self.max_vel_ang = rospy.get_param('~max_vel_ang', 1.82)

        self.target = Twist()
        rospy.Subscriber('command', Int32, self.update_target)

        self.publisher_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        rospy.Timer(rospy.Duration(rospy.get_param('~freq', 0.1)), self.spin)

    def update_target(self, data):
        """목표를 갱신한다"""
        if data.data == 0:      # left
            self.target.linear.x = 0
            self.target.angular.z = self.max_vel_ang
        elif data.data == 1:    # forward
            self.target.linear.x = self.max_vel_lin
            self.target.angular.z = 0
        elif data.data == 2:    # right
            self.target.linear.x = 0
            self.target.angular.z = -self.max_vel_ang
        elif data.data == 3:    # stop
            self.target.linear.x = 0
            self.target.angular.z = 0
        elif data.data == 4:    # backward
            self.target.linear.x = -self.max_vel_lin
            self.target.angular.z = 0

    def spin(self, event):
        """목표를 실행한다"""
        twist = copy.copy(self.target)
        self.publisher_cmd_vel.publish(twist)


if __name__ == '__main__':
    rospy.init_node('local_path_planner')
    lpp = LocalPathPlanner()
    rospy.spin()
