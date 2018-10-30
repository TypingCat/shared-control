#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class JOYSTICK:
    """조이스틱 메시지를 관리한다"""
    def __init__(self):
        self.cmd_vel = Twist()
        self.cmd_sel = 0
        self.vel_lin_max = rospy.get_param('~velocity_lin', 0.26)
        self.vel_ang_max = rospy.get_param('~velocity_ang', 1.82)

        rospy.Subscriber('joy', Joy, self.update_cmd)

        self.publisher_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.publisher_cmd_sel = rospy.Publisher('cmd_sel', Int32, queue_size=1)

    def update_cmd(self, data):
        """조이스틱 메시지를 제어명령으로 전환한다"""
        self.cmd_vel.linear.x = self.vel_lin_max*data.axes[1]
        self.cmd_vel.angular.z = self.vel_ang_max*data.axes[0]

        if data.buttons[0] == 1:
            self.cmd_sel = 1
        elif data.buttons[1] == 1:
            self.cmd_sel = 2
        elif data.buttons[2] == 1:
            self.cmd_sel = 3
        elif data.buttons[3] == 1:
            self.cmd_sel = 4
        else:
            self.cmd_sel = 0

        self.publisher_cmd_vel.publish(self.cmd_vel)
        self.publisher_cmd_sel.publish(self.cmd_sel)


if __name__ == '__main__':
    rospy.init_node('joystick')
    joystick = JOYSTICK()
    rospy.spin()
