#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import actionlib
import copy

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MOTION_MANAGER:
    """이동로봇의 움직임을 관리한다."""
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.state = 0

        rospy.Subscriber('robot/target', Pose, self.update_target)

        self.publisher_state = rospy.Publisher('robot/state', Int32, queue_size=1)

        rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.test)

        rospy.loginfo('초기화 완료')

    def update_target(self, data):
        goal = MoveBaseGoal()                                       # 목표를 설정한다.
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = data.position.x
        goal.target_pose.pose.position.y = data.position.y
        goal.target_pose.pose.position.z = data.position.z
        goal.target_pose.pose.orientation.x = data.orientation.x
        goal.target_pose.pose.orientation.y = data.orientation.y
        goal.target_pose.pose.orientation.z = data.orientation.z
        goal.target_pose.pose.orientation.w = data.orientation.w

        self.client.send_goal(goal)
        self.state = 1
        self.client.wait_for_result()
        self.state = 0

    def test(self, event):
        self.publisher_state.publish(self.state)
        # rospy.loginfo(self.state)


if __name__ == '__main__':
    rospy.init_node('motion_manager')
    motion_manager = MOTION_MANAGER()
    rospy.spin()
