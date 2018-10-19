#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import math
import tf

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32

from shared_control.srv import Nearest, Neighbors, Node, MotorImagery


class TASK_PLANNER:
    """로봇의 행동방침을 결정한다"""
    def __init__(self):
        self.move = 0               # 로봇의 움직임: 0=정지, 1=이동중
        self.pose = Pose()          # 로봇의 자세
        self.nearest = [-1, 0]      # 로봇과 가장 가까운 노드
        self.history = [-1, -1]     # 목표노드 기록
        self.questions = []         # 질문 목록
        self.state = 0              # 상태: -1=수면, 0=대기, 1=이벤트, 2=트리거

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
        rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.explosion)

        rospy.loginfo('')
        rospy.loginfo('준비되었습니다. Eyeblink는 \'w\', motorimagery는 \'a(적색), d(청색)\'입니다.')

    def mount_gvg(self):
        """이동로봇을 GVG 위로 이동시킨다"""
        rospy.sleep(rospy.get_param('~planning_cycle', 0.5))

        nearest = self.get_nearest(self.pose.position).id
        if nearest != -1:   # 가장 가까운 노드로 이동한다.
            self.send_target(nearest)
            self.history = [nearest, nearest]

        else:               # 성공할 때까지 초기화를 시도한다.
            self.mount_gvg()

    def planning(self, event):
        """움직임을 계획한다"""
        self.questions = []
        if self.state == -1:                                # 수면중이었다면,
            options = self.get_options(self.history[0])     # 질문을 생성한다.
            for id in options:
                self.questions.append(id)

        elif self.state == 1:                               # 노드 위에 정지해 있다면,
            options = self.get_options(self.history[0])
            try:
                options.remove(self.history[1])
            except: pass

            if len(options) == 0:                           # 말단이라면 수면을 취한다.
                self.state = -1
                rospy.loginfo('대기합니다.')
                return

            else:                                           # 교차로라면 질문을 생성한다.
                for id in options:
                    self.questions.append(id)

        elif self.state == 2:                               # 트리거를 받았을 때,
            if self.move == 0:                              # 정지해 있다면,
                rospy.loginfo('이럴리가 없는데?')

            else:                                           # 이동중이라면,
                self.target_publisher.publish(self.pose)    # 정지한다.
                self.questions.append(self.history[0])      # 그리고 앞으로 갈지 뒤로갈지를 결정한다.
                self.questions.append(self.history[1])

        answer = -1
        for question in self.questions:                     # Motorimagery로 질문한다.
            self.send_target(question, 1)                   # 목적지를 보여준다.

            answer = self.get_motorimagery((question, -1)).id
            if answer != -1:
                break
            else:
                answer = -2
                rospy.sleep(rospy.get_param('~planning_cycle', 0.5))

        if answer > -1:                 # 답변을 얻었다면,
            self.send_target(answer)    # 이동한다.
            rospy.loginfo('%d로 이동합니다.'%answer)

        if answer == -2:                # 답변이 없었다면,
            self.state = -1             # 수면에 든다.
            rospy.loginfo('대기합니다.')

        else:                           # 질문이 없었다면,
            self.state = 0              # 대기한다.
            rospy.loginfo('대기합니다.')

    def get_options(self, id):
        """그래프로부터 선택지를 확인한다"""
        options = list(self.get_neighbors(id).ids)      # 이웃노드가 선택지이다.
        try:
            options.remove(self.history[0])             # 방문한 노드들은 제외한다.
        except: pass

        return options

    def send_target(self, id, head_only=0):
        """이동로봇에게 목표자세를 전송한다"""
        pose = Pose()
        pose.position = self.get_node(id).point         # 위치를 설정한다.

        dx = pose.position.x - self.pose.position.x     # 방향을 설정한다.
        dy = pose.position.y - self.pose.position.y
        th = math.atan2(dy, dx)
        q = tf.transformations.quaternion_from_euler(0, 0, th)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        if head_only == 1:                              # 쳐다보기만 할지 결정한다.
            pose.position.x = self.pose.position.x
            pose.position.y = self.pose.position.y
            pose.position.z = self.pose.position.z

        self.target_publisher.publish(pose)             # 목표를 발행한다.
        self.history[1] = self.history[0]               # 기록한다.
        self.history[0] = id

    def explosion(self, event):
        """이동로봇이 노드에 도달했는지를 감시한다"""
        if (self.state == 0)&\
           (self.move == 0)&\
           (self.history[0] == self.nearest[0])&\
           (self.nearest[1] < 2*rospy.get_param('~goal_margin', 0.01)):
            self.state = 1
            rospy.Timer(rospy.Duration(rospy.get_param('~planning_cycle', 0.5)), self.planning, oneshot=True)

    def percussion(self, data):
        """트리거가 발생하면 이동목표를 갱신한다"""
        if self.state == 0:
            self.state = 2
            rospy.Timer(rospy.Duration(rospy.get_param('~planning_cycle', 0.5)), self.planning, oneshot=True)
        elif self.state == -1:
            rospy.Timer(rospy.Duration(rospy.get_param('~planning_cycle', 0.5)), self.planning, oneshot=True)
        else:
            rospy.loginfo("잠깐만요.")

    def update_state(self, data):
        """로봇의 상태를 갱신한다"""
        self.move = data.data

    def update_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.pose = data                                                # 현재 자세를 갱신한다.

        try:
            self.nearest[0] = self.get_nearest(self.pose.position).id   # 가장 가까운 노드를 갱신한다.
            p = self.get_node(self.nearest[0]).point
            self.nearest[1] = (p.x-self.pose.position.x)**2 + (p.y-self.pose.position.y)**2
        except: pass


if __name__ == '__main__':
    rospy.init_node('task_planner')
    task_planner = TASK_PLANNER()
    rospy.spin()
