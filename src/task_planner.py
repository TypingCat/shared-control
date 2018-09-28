#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32

from shared_control.srv import Nearest, Neighbors, Node, MotorImagery


class TASK_PLANNER:
    """로봇의 행동방침을 결정한다"""
    def __init__(self):
        self.state = 0                          # 로봇의 상태: 0=정지, 1=이동중
        self.pose = Pose()                      # 로봇의 자세
        self.history = [-1, -1]                 # 목표노드 기록
        self.questions = []                     # 질문 목록

        rospy.Subscriber('bci/eyeblink', Int32, self.percussion, queue_size=1)
        rospy.Subscriber('robot/state', Int32, self.update_state)
        rospy.Subscriber('robot/pose', Pose, self.update_pose)

        self.target_publisher = rospy.Publisher('robot/target', Pose, queue_size=1)

        rospy.wait_for_service('gvg/nearest')   # 서비스들을 등록한다.
        rospy.wait_for_service('gvg/neighbors')
        rospy.wait_for_service('gvg/node')
        rospy.wait_for_service('bci/motorimagery')
        self.get_nearest = rospy.ServiceProxy('gvg/nearest', Nearest)
        self.get_neighbors = rospy.ServiceProxy('gvg/neighbors', Neighbors)
        self.get_node = rospy.ServiceProxy('gvg/node', Node)
        self.get_motorimagery = rospy.ServiceProxy('bci/motorimagery', MotorImagery)

        self.mount_gvg()                        # 이동을 시작한다.
        rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.spin)

        rospy.loginfo('')
        rospy.loginfo('준비되었습니다. Eyeblink는 \'w\', motorimagery는 \'a(적색), d(청색)\'입니다.')

    def mount_gvg(self):
        """이동로봇을 gvg 위로 이동시킨다"""
        rospy.sleep(rospy.get_param('~planning_cycle', 0.5))

        target = self.get_nearest(self.pose.position)
        if target.id != -1:     # 가장 가까운 노드로 이동한다.
            pose = Pose()
            pose.position = self.get_node(target.id).point
            self.target_publisher.publish(pose)
            self.history = [target.id, target.id]

        else:                   # 노드 확인에 실패하면 다시 시도한다.
            self.mount_gvg()

    def spin(self, event):
        """이동로봇의 좌표를 감시한다"""


        pass


    def percussion(self, data):
        """트리거가 발생하면 이동목표를 갱신한다"""
        if self.state == 0:     # 로봇의 상태에 따른 선택지를 확인한다.
            options = list(self.get_neighbors(self.history[0]).ids)
            try:
                options.remove(self.history[1])
            except: pass
        elif self.state == 1:
            self.target_publisher.publish(self.pose)
            options = self.history

        self.questions = []
        if len(options) == 1:   # 선택지가 하나라면 이동할지 말지를 고려한다.
            self.questions = [(options[0], -1)]
        elif len(options) == 2: # 선택지가 둘이라면 둘 중 하나를 선택해야 한다.
            self.questions = [tuple(options)]
        else:                   # 그 외에는 질문을 binary로 변환한다.
            for id in options:
                self.questions.append((id, -1))

        rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.questioning, oneshot=True)

    def questioning(self, event):
        """질문하고 답변을 대기한다"""
        for question in self.questions:
            answer = self.get_motorimagery(question).id
            if answer != -1:
                pose = Pose()
                pose.position = self.get_node(answer).point
                self.target_publisher.publish(pose)
                self.history[1] = self.history[0]
                self.history[0] = answer

                rospy.loginfo('%d로 이동합니다.'%answer)
                return

            rospy.sleep(rospy.get_param('~planning_cycle', 0.5))

        rospy.loginfo('대기합니다.')

    def update_state(self, data):
        """로봇의 상태를 갱신한다"""
        self.state = data.data

    def update_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.pose = data


if __name__ == '__main__':
    rospy.init_node('task_planner')
    task_planner = TASK_PLANNER()
    rospy.spin()
