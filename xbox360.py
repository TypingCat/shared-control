#!/usr/bin/python
#-*-coding:utf-8-*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

class RosJoystick():

    def __init__(self):
        rospy.Subscriber("joy", Joy, self.callback)
        self.joystick_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
        self.button_pub = rospy.Publisher('button', Int32MultiArray, queue_size=10)

    def callback(self, msg):
        # 조이스틱 리스트 정의
        # 1번째: 왼쪽 조이스틱의 수평축(서쪽이 +), 2번째: 왼쪽 조이스틱의 수직축(북쪽이 +)
        # 3번째: 오른쪽 조이스틱의 수평축(서쪽이 +), 4번째: 오른쪽 조이스틱의 수직축(북쪽이 +)
        # 5번째, 6번째: LT&RT키 관련 필요 없음,
        # 7번째: 방향키의 수평축(서쪽이 +), 8번째: 방향키의 수직축(북쪽이 +)
        # 버튼 리스트 정의
        # 1번째: A, 2번째: B, 3번째: X, 4번째: Y
        # 5번째, 6번째: LB&RB키 관련 필요 없음,
        # 7번째: BACK, 8번째: START
        # 9번째: XBOX키, 10번째: 왼쪽 조이스틱 누름, 11번째: 오른쪽 조이스틱 누름
        button = Int32MultiArray()
        button.data = msg.buttons
        self.button_pub.publish(button)
        twist = Twist()
        twist.linear.x = 4 * msg.axes[1]
        twist.angular.z = 4 * msg.axes[0]
        self.joystick_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('ros_joystick')
    ros_joystick = RosJoystick()
    rospy.spin()