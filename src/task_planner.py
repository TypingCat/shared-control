#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import math
import tf

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32

from shared_control.srv import Nearest, Neighbors, Node, MotorImagery


class TASK_PLANNER:
    """로봇의 행동방침을 결정한다"""
    def __init__(self):
        self.state = 0                          # 로봇의 상태: 0=정지, 1=이동중
        self.pose = Pose()                      # 로봇의 자세
        self.nearest = [-1, 0]                  # 로봇과 가장 가까운 노드
        self.history = [-1, -1]                 # 목표노드 기록
        self.questions = []                     # 질문 목록
        self.busy = 0

        rospy.wait_for_service('gvg/nearest')   # 초기화를 기다린다.
        rospy.wait_for_service('gvg/neighbors')
        rospy.wait_for_service('gvg/node')
        rospy.wait_for_service('bci/motorimagery')
        rospy.sleep(1.0)

        rospy.Subscriber('bci/eyeblink', Int32, self.percussion, queue_size=1)
        rospy.Subscriber('robot/state', Int32, self.update_state)
        rospy.Subscriber('robot/pose', Pose, self.update_pose)

        self.target_publisher = rospy.Publisher('robot/target', Pose, queue_size=1)

        self.get_nearest = rospy.ServiceProxy('gvg/nearest', Nearest)   # 서비스들을 등록한다.
        self.get_neighbors = rospy.ServiceProxy('gvg/neighbors', Neighbors)
        self.get_node = rospy.ServiceProxy('gvg/node', Node)
        self.get_motorimagery = rospy.ServiceProxy('bci/motorimagery', MotorImagery)

        self.mount_gvg()                                                # 이동을 시작한다.
        rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.spin)

        rospy.loginfo('')
        rospy.loginfo('준비되었습니다. Eyeblink는 \'w\', motorimagery는 \'a(적색), d(청색)\'입니다.')

    def mount_gvg(self):
        """이동로봇을 gvg 위로 이동시킨다"""
        rospy.sleep(rospy.get_param('~planning_cycle', 0.5))

        nearest = self.get_nearest(self.pose.position).id
        if nearest != -1:       # 가장 가까운 노드로 이동한다.
            pose = Pose()
            pose.position = self.get_node(nearest).point    # 위치를 설정한다.

            dx = pose.position.x - self.pose.position.x     # 방향을 설정한다.
            dy = pose.position.y - self.pose.position.y
            th = math.atan2(dy, dx)
            q = tf.transformations.quaternion_from_euler(0, 0, th)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            self.target_publisher.publish(pose)             # 목표를 발행한다.
            self.history = [nearest, nearest]

        else:                   # 노드 확인에 실패하면 다시 시도한다.
            self.mount_gvg()

    def spin(self, event):
        """이동로봇의 좌표를 감시한다"""
        self.planning()

        # if (self.nearest[1] < rospy.get_param('~goal_margin', 0.01)) &\
        #    (self.history[0] == self.nearest[0]):                    # 노드에 도달했다면,
        #     options = list(self.get_neighbors(self.nearest[0]).ids) # 이웃노드가 다음 선택지이다.
        #     try:
        #         options.remove(self.nearest[0])
        #         options.remove(self.history[1])
        #     except: pass
        # else:
        #     return
        #
        # self.questions = []
        # if len(options) == 0:   # 선택지가 하나라면 이동할지 말지를 고려한다.
        #     self.questions = [(self.history[1], -1)]
        # else:                   # 그 외에는 질문을 binary로 변환한다.
        #     for id in options:
        #         self.questions.append((id, -1))
        #
        # rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.questioning, oneshot=True)

    def percussion(self, data):
        """트리거가 발생하면 이동목표를 갱신한다"""
        self.planning()

        # self.questions = []
        # if self.state == 0:                                          # 로봇이 정지해 있을 경우,
        #     if self.nearest[1] < rospy.get_param('~goal_margin', 0.01): # 로봇이 노드에 놓여있다면,
        #         options = list(self.get_neighbors(self.nearest[0]).ids) # 이웃노드가 선택지이다.
        #         try:
        #             options.remove(self.nearest[0])
        #             options.remove(self.history[1])
        #         except: pass
        #     else:                           # 엣지에 놓여있다면,
        #         options = self.history      # 히스토리가 선택지이다.
        #
        #     if len(options) == 0:           # 선택지가 하나라면 이전 위치로 이동할지 말지를 고려한다.
        #         self.questions = [(self.history[1], -1)]
        #     else:                           # 그 외에는 질문을 binary로 변환한다.
        #         for id in options:
        #             self.questions.append((id, -1))
        #
        # elif self.state == 1:                        # 로봇이 이동중일 경우,
        #     self.target_publisher.publish(self.pose)    # 일단 정지한다.
        #     options = self.history                      # 히스토리가 선택지이다.
        #
        #     self.questions = [tuple(options)]
        #
        # else:
        #     return
        #
        # rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.questioning, oneshot=True)

    def planning(self):
        if self.busy == 1:  # 중복실행을 방지한다.
            return
        else:
            self.busy = 1





        self.nearest[0] = self.get_nearest(self.pose.position).id   # 가장 가까운 노드를 갱신한다.
        p = self.get_node(self.nearest[0]).point
        self.nearest[1] = (p.x-self.pose.position.x)**2 + (p.y-self.pose.position.y)**2



        self.busy = 0

    # def questioning(self, event):
    #     """질문하고 답변을 대기한다"""
    #     if self.questioning_busy == 1:  # 질문은 중복하지 않는다.
    #         return
    #     else:
    #         self.questioning_busy = 1
    #
    #     for question in self.questions: # Motorimagery 질문을 생성한다.
    #         answer = self.get_motorimagery(question).id
    #         if answer != -1:
    #             pose = Pose()
    #             pose.position = self.get_node(answer).point     # 위치를 설정한다.
    #
    #             dx = pose.position.x - self.pose.position.x     # 방향을 설정한다.
    #             dy = pose.position.y - self.pose.position.y
    #             th = math.atan2(dy, dx)
    #             q = tf.transformations.quaternion_from_euler(0, 0, th)
    #             pose.orientation.x = q[0]
    #             pose.orientation.y = q[1]
    #             pose.orientation.z = q[2]
    #             pose.orientation.w = q[3]
    #
    #             self.target_publisher.publish(pose)             # 목표를 발행한다.
    #             self.history[1] = self.history[0]               # 기록한다.
    #             self.history[0] = answer
    #
    #             rospy.loginfo('%d로 이동합니다.'%answer)
    #             self.questioning_busy = 0
    #             return
    #
    #         rospy.sleep(rospy.get_param('~planning_cycle', 0.5))
    #
    #     rospy.loginfo('대기합니다.')
    #     self.questioning_busy = 0

    def update_state(self, data):
        """로봇의 상태를 갱신한다"""
        self.state = data.data

    def update_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.pose = data                                            # 현재 자세를 갱신한다.

        # self.nearest[0] = self.get_nearest(self.pose.position).id   # 가장 가까운 노드를 갱신한다.
        # p = self.get_node(self.nearest[0]).point
        # self.nearest[1] = (p.x-self.pose.position.x)**2 + (p.y-self.pose.position.y)**2


if __name__ == '__main__':
    rospy.init_node('task_planner')
    task_planner = TASK_PLANNER()
    rospy.spin()
