#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import actionlib

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MOTION_MANAGER:
    """이동로봇의 움직임을 관리한다."""
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.seq = [0, 0]

        rospy.Subscriber('robot/target', Pose, self.update_target)

        self.publisher_state = rospy.Publisher('robot/state', Int32, queue_size=1)

        rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.control)

    def update_target(self, data):
        self.seq[1] = rospy.get_time()          # 목표를 기록한다.
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = data

    def control(self, event):
        if self.seq[0] != self.seq[1]:          # 새로운 목표가 확인된다면,
            self.seq[0] = self.seq[1]           # 목표를 갱신한다.
            self.client.cancel_goal()
            self.client.wait_for_server()
            self.client.send_goal(self.goal)

        if self.seq[0] == 0:                    # 현재상태를 발행한다.
            self.publisher_state.publish(0)     # simple_action_client: ACTIVE(1)
        elif self.client.get_state() != 1:
            self.publisher_state.publish(0)
        else:
            self.publisher_state.publish(1)


if __name__ == '__main__':
    rospy.init_node('motion_manager')
    motion_manager = MOTION_MANAGER()
    rospy.spin()
