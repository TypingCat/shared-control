#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import math
import tf

from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int32

from shared_control.msg import MID
from shared_control.srv import Nearest, Neighbors, Node, MotorImagery


class TASK_PLANNER:
    """로봇의 행동방침을 결정한다"""
    def __init__(self):
        self.eyeblink = rospy.get_time()

        rospy.wait_for_service('gvg/nearest')   # 서비스 초기화를 기다린다.
        rospy.wait_for_service('gvg/neighbors')
        rospy.wait_for_service('gvg/node')
        rospy.wait_for_service('bci/motorimagery')

        rospy.Subscriber('bci/eyeblink', Int32, self.percussion)
        rospy.Subscriber('robot/state', Int32, self.update_state)
        rospy.Subscriber('robot/pose', Pose, self.update_pose)

        self.publisher_target = rospy.Publisher('robot/target', Pose, queue_size=1)
        self.publisher_douser = rospy.Publisher('interface/douser', Int32, queue_size=1)
        self.publisher_MID_L = rospy.Publisher('interface/MID_L', MID, queue_size=1)
        self.publisher_MID_R = rospy.Publisher('interface/MID_R', MID, queue_size=1)
        self.publisher_MID_confirm = rospy.Publisher('interface/MID_confirm', MID, queue_size=1)

        self.get_nearest = rospy.ServiceProxy('gvg/nearest', Nearest)   # 서비스들을 등록한다.
        self.get_neighbors = rospy.ServiceProxy('gvg/neighbors', Neighbors)
        self.get_node = rospy.ServiceProxy('gvg/node', Node)
        self.get_motorimagery = rospy.ServiceProxy('bci/motorimagery', MotorImagery)

        self.mount_gvg()                                                # 이동을 시작한다.
        rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.explosion)
        rospy.loginfo('준비되었습니다. Eyeblink는 \'s\', motorimagery는 \'a, d\'입니다.')

    def mount_gvg(self):
        """이동로봇을 GVG 위로 이동시킨다"""
        nearest = -1
        while nearest == -1:
            try:                                # 가장 가까운 노드를 검색한다.
                nearest = self.get_nearest(self.pose.position).id
            except:
                rospy.sleep(rospy.get_param('~planning_cycle', 0.5))
        self.history = [nearest, nearest, nearest, -1]         # 목표, 현재, 최근, 이전

        self.send_target(nearest)               # 가장 가까운 노드로 이동한다.
        self.state = -1

    def percussion(self, data):
        """트리거가 발생하면 이동목표를 갱신한다"""

        # if self.state == 0:
        #     self.state = 2
        #     rospy.Timer(rospy.Duration(rospy.get_param('~planning_cycle', 0.5)), self.planning, oneshot=True)

        if self.state == -1:    # 휴면 상태인 로봇을 깨운다.
            self.state = 0
            # rospy.Timer(rospy.Duration(rospy.get_param('~planning_cycle', 0.5)), self.planning, oneshot=True)

            neighbors = list(self.get_neighbors(self.history[1]).ids)
            if len(neighbors) == 1:                         # 길이 하나밖에 없다면,
                rospy.loginfo("일단 이동합니다.")              # 질문할 필요 없이 이동한다.

                self.send_target(neighbors[0])

        else:                   # 아니면 시간을 기록한다.
            self.eyeblink = rospy.get_time()

        # else:
        #     rospy.loginfo("잠깐만요(state=%d)"%self.state)

    def explosion(self, event):
        """이동로봇의 임무계획시점을 판단한다"""
        need_plan = False       # 노드에 도달하여 대기중이라면 다음 움직임을 계획한다.
        try:                    # 단 초기화가 완료될 때까지는 실행하지 않는다.
            if (self.state == 0)&\
               (self.move == 0)&\
               (self.history[0] == self.history[1]):
               need_plan = True
        except:
            return

        # rospy.loginfo([self.state, self.move, self.history[0], self.history[1], self.history[2]])

        if need_plan:           # 계획을 시작한다.
            self.state = 1
            self.publisher_douser.publish(self.state)


            rospy.loginfo('계획을 시작하지')

            # self.history[0] = 1
            # self.state = 1
            rospy.Timer(rospy.Duration(rospy.get_param('~planning_cycle', 0.5)), self.planning, oneshot=True)

    #     if (self.state == 0)&\
    #        (self.move == 0)&\
    #        (self.history[0] == self.nearest[0]):
    #
    #         self.count = self.count + 1     # 노드에 도달했는지 신중히 확인한다.
    #         if self.count > 4:
    #             self.state = 1      # 노드에 도달했다면 이동목표를 갱신한다.
    #             rospy.Timer(rospy.Duration(rospy.get_param('~planning_cycle', 0.5)), self.planning, oneshot=True)
    #             self.count = 0
        elif (self.state == 2)&\
             (self.move == 0)&\
             (self.history[0] == self.history[1]):    # 목표에 도달했음을 기록한다.
            self.state = 0
            self.history[3] = self.history[2]
            self.history[2] = self.history[1]

    def planning(self, event):
        """움직임을 계획한다"""
        # self.questions = []
        # if self.state == -1:                                # 수면중이었다면,
        #     options = self.get_options(self.history[0])     # 질문을 생성한다.
        #     for id in options:
        #         self.questions.append(id)

        # if self.state == 1:                     # 노드 위에 정지해 있다면,
        # rospy.loginfo(self.history)

        neighbors = list(self.get_neighbors(self.history[2]).ids)
        try:                                            # 지나온 노드를 제외한 이웃을 검색한다.
            neighbors.remove(self.history[3])
        except: pass


        # rospy.loginfo([self.state, self.move, self.history[0], self.history[1], self.history[2], self.history[3]])
        # rospy.loginfo(neighbors)


        if len(neighbors) == 0:                         # 막다른 길일 경우,
            rospy.loginfo("대기합니다.")                  # 휴면상태로 전환한다.

            # self.history[1] = self.history[0]
            self.state = -1

        elif len(neighbors) == 1:                       # 길이 하나밖에 없다면,
            rospy.loginfo("일단 이동합니다.")              # 질문할 필요 없이 이동한다.

            self.send_target(neighbors[0])

        elif len(neighbors) == 2:                       # 길이 두개라면,
            rospy.loginfo("방향을 선택해 주세요.")           # MI로 질문한다.

            p0 = self.get_node(self.history[1]).point
            p1 = self.get_node(neighbors[0]).point
            p2 = self.get_node(neighbors[1]).point
            dth, th1, th2 = self.head_target(p0, p1, p2)
            if dth > 0:
                neighbors[0], neighbors[1] = neighbors[1], neighbors[0]
                th1, th2 = th2, th1

            left = MID()                                # 선택지 마커를 출력한다.
            left.point = p0
            left.th = th1
            self.publisher_MID_L.publish(left)
            right = MID()
            right.point = p0
            right.th = th2
            self.publisher_MID_R.publish(right)

            answer = self.get_motorimagery((neighbors[0], neighbors[1])).id
            if answer == neighbors[0]:                  # MI 서비스를 요청한다.
                self.prearrangement = [0, neighbors[0], neighbors[1], self.history[3]]
            else:
                self.prearrangement = [0, neighbors[1], neighbors[0], self.history[3]]
                th1, th2 = th2, th1

            direction = MID()                           # 결과를 마커로 출력한다.
            direction.point = p0
            direction.th = th1
            self.publisher_MID_confirm.publish(direction)


            rospy.loginfo("가즈아!")
            rospy.loginfo(self.history)
            rospy.loginfo(neighbors)


            rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.go_around, oneshot=True)             # 행동을 취한다.

        else:
            rospy.loginfo("갈림길이... 너무 많은데요?")
            rospy.loginfo(self.history)
            rospy.loginfo(neighbors)

            self.state = -1


            # options = self.get_options(self.history[0])
            # try:
            #     options.remove(self.history[1])
            #     # dist = math.sqrt((self.dst.x - self.pose.x)**2 +
            #     #                  (self.dst.y - self.pose.y)**2)
            #     # rospy.loginfo(dist)
            #     # if dist > self.dst.z:
            #     #     rospy.loginfo("지나온 노드 제거")
            #     #     options.remove(self.history[1])
            # except: pass
            #
            # if len(options) == 0:                           # 말단이라면 수면을 취한다.
            #     self.state = -1
            #     self.publisher_douser.publish(self.state)
            #     rospy.loginfo('대기합니다.')
            #     return
            #
            # else:                                           # 교차로라면 질문을 생성한다.
            #     for id in options:
            #         self.questions.append(id)


    #     elif self.state == 2:                               # 트리거를 받았을 때,
    #         if self.move == 0:                              # 정지해 있다면,
    #             self.publisher_douser.publish(self.state)
    #             rospy.loginfo('이럴리가 없는데?(state=%d)'%self.state)
    #
    #         else:                                           # 이동중이라면,
    #             self.publisher_target.publish(self.pose)    # 정지한다.
    #             self.questions.append(self.history[0])      # 그리고 앞으로 갈지 뒤로갈지를 결정한다.
    #             self.questions.append(self.history[1])
    #
    #     answer = -1
    #     for question in self.questions:                     # Motorimagery로 질문한다.
    #         self.send_target(question, 1)                   # 로봇의 방향각을 목적지로 향한다.
    #
    #         answer = self.get_motorimagery((question, -1)).id
    #         if answer != -1:
    #             break
    #         else:
    #             answer = -2
    #             rospy.sleep(rospy.get_param('~planning_cycle', 0.5))
    #
    #     if answer > -1:                 # 답변을 얻었다면,
    #         self.state = 0
    #         self.send_target(answer)    # 이동한다.
    #
    #     elif answer == -1:              # 답변이 없었다면,
    #         self.state = 0              # 대기한다.
    #         self.publisher_douser.publish(self.state)
    #         rospy.loginfo('대기합니다.')
    #
    #     elif answer == -2:              # 질문 자체가 없었다면,
    #         self.state = -1             # 수면에 든다.
    #         self.publisher_douser.publish(self.state)
    #         rospy.loginfo('대기합니다.')
    #
    #     else:
    #         rospy.loginfo('네?(state=%d)'%self.state)

    # def get_options(self, id):
    #     """그래프로부터 선택지를 확인한다"""
    #     options = list(self.get_neighbors(id).ids)      # 이웃노드가 선택지이다.
    #     try:
    #         options.remove(self.history[0])             # 방문한 노드들은 제외한다.
    #     except: pass
    #
    #     return options

    def go_around(self, event):
        """명령을 행동으로 옮긴다"""
        # time_start = rospy.get_time()
        # while (rospy.get_time() - time_start < self.patience) and (self.eyeblink - time_start < 0):
        #     rospy.sleep(rospy.get_param('~spin_cycle', 0.1))
        # if self.eyeblink - time_start > 0:
        #     self.prearrangement[0], self.prearrangement[1] = self.prearrangement[1], self.prearrangement[0]



        self.send_target(self.prearrangement[1])

        rospy.loginfo(self.prearrangement)

        now = rospy.get_time()
        while self.state == 2:                  # 목표에 도달하기 전에,
            if self.eyeblink > now:             # eyeblink가 들어오면 목표를 변경한다.
                self.eyeblink = now
                self.prearrangement[0] = self.prearrangement[0] + 1
                self.send_target(self.prearrangement[self.prearrangement[0]%3+1])

            rospy.sleep(rospy.get_param('~spin_cycle', 0.1))

            # if self.eyeblink - time_start > 0:
            #     self.prearrangement[0], self.prearrangement[1] = self.prearrangement[1], self.prearrangement[0]


        # self.publisher_douser.publish(self.prearrangement[0])
        # self.state = 0

    def send_target(self, id):
        """이동로봇에게 목표자세를 전송한다"""
        pose = Pose()
        pose.position = self.get_node(id).point                 # 위치를 설정한다.

        dx = pose.position.x - self.pose.position.x             # 방향을 설정한다.
        dy = pose.position.y - self.pose.position.y
        th = math.atan2(dy, dx)
        q = tf.transformations.quaternion_from_euler(0, 0, th)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.publisher_target.publish(pose)                     # 목표를 발행한다.
        self.history[0] = id
        self.state = 2

        # rospy.sleep(rospy.get_param('~planning_cycle', 0.5))    # 쿨타임을 적용한다.

    def head_target(self, p0, p1, p2):
        """로봇이 특정 각도를 바라보도록 한다"""
        th1 = math.atan2(p1.y-p0.y, p1.x-p0.x)          # 사잇각을 계산한다.
        th2 = math.atan2(p2.y-p0.y, p2.x-p0.x)
        dth = (th2 - th1 + math.pi)%(2*math.pi) - math.pi
        th = th1 + dth/2

        pose = Pose()                                   # 자세를 결정한다.
        pose.position = p0
        q = tf.transformations.quaternion_from_euler(0, 0, th)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.publisher_target.publish(pose)             # 목표를 발행한다.
        return dth, th1, th2





    def update_state(self, data):
        """로봇의 상태를 갱신한다"""
        self.move = data.data

    def update_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.pose = data                                                # 현재 자세를 갱신한다.
        try:
            self.history[1] = self.get_nearest(self.pose.position).id   # 가장 가까운 노드를 갱신한다.
            # p = self.get_node(self.nearest[0]).point
            # self.nearest[1] = (p.x-self.pose.position.x)**2 + (p.y-self.pose.position.y)**2
        except: pass

    # def update_dst(self, data):
    #     """목적지를 갱신한다"""
    #     self.dst.x = data.x
    #     self.dst.y = data.y
    #     self.dst.z = data.z

if __name__ == '__main__':
    rospy.init_node('task_planner')
    task_planner = TASK_PLANNER()
    rospy.spin()
