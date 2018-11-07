#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import math
import tf

from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int32

from shared_control.srv import Nearest, Neighbors, Node, MotorImagery


class TASK_PLANNER:
    """로봇의 행동방침을 결정한다"""
    def __init__(self):
        # self.move = 1               # 로봇의 움직임: 0=정지, 1=이동중
        # self.pose = PoseStamped()          # 로봇의 자세
        # self.nearest = 0      # 로봇과 가장 가까운 노드
        # self.history = [-1, -1]     # 목표노드 기록
        # self.questions = []         # 질문 목록
        self.state = -1              # 상태: -1=수면, 0=대기, 1=이벤트, 2=트리거
        # self.count = 0
        # self.dst = Point()
        # self.init_confirm = [0, 0]

        rospy.wait_for_service('gvg/nearest')   # 서비스 초기화를 기다린다.
        rospy.wait_for_service('gvg/neighbors')
        rospy.wait_for_service('gvg/node')
        rospy.wait_for_service('bci/motorimagery')

        rospy.Subscriber('bci/eyeblink', Int32, self.percussion)
        rospy.Subscriber('robot/state', Int32, self.update_state)
        rospy.Subscriber('robot/pose', Pose, self.update_pose)
        # rospy.Subscriber('interface/destination', Point, self.update_dst)

        self.publisher_target = rospy.Publisher('robot/target', Pose, queue_size=1)
        # self.publisher_lighter = rospy.Publisher('interface/lighter', Point, queue_size=1)
        # self.publisher_flicker = rospy.Publisher('interface/flicker', Point, queue_size=1)
        # self.publisher_douser = rospy.Publisher('interface/douser', Int32, queue_size=1)

        self.get_nearest = rospy.ServiceProxy('gvg/nearest', Nearest)   # 서비스들을 등록한다.
        self.get_neighbors = rospy.ServiceProxy('gvg/neighbors', Neighbors)
        self.get_node = rospy.ServiceProxy('gvg/node', Node)
        self.get_motorimagery = rospy.ServiceProxy('bci/motorimagery', MotorImagery)

        self.mount_gvg()                                                # 이동을 시작한다.
        rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.explosion)

        rospy.loginfo('')
        rospy.loginfo('준비되었습니다. Eyeblink는 \'s\', motorimagery는 \'a, d\'입니다.')

    def mount_gvg(self):
        """이동로봇을 GVG 위로 이동시킨다"""
        nearest = -1
        while nearest == -1:
            try:                    # 가장 가까운 노드를 검색한다.
                nearest = self.get_nearest(self.pose.position).id
            except:
                rospy.sleep(rospy.get_param('~planning_cycle', 0.5))

        self.history = [nearest, nearest]
        self.send_target(nearest)   # 가장 가까운 노드로 이동한다.

    def planning(self, event):
        """움직임을 계획한다"""
        # self.questions = []
        # if self.state == -1:                                # 수면중이었다면,
        #     options = self.get_options(self.history[0])     # 질문을 생성한다.
        #     for id in options:
        #         self.questions.append(id)

        if self.state == 1:                             # 노드 위에 정지해 있다면,
            neighbors = list(self.get_neighbors(self.history[0]).ids)
            try:
                neighbors.remove(self.history[1])
            except: pass

            if len(neighbors) == 0:
                rospy.loginfo("그래프가 이상해!")

            elif len(neighbors) == 1:                   # 길이 하나밖에 없다면 이동한다.
                self.send_target(neighbors[0])
                self.state = 0
                rospy.sleep(rospy.get_param('~planning_cycle', 0.5))

            elif len(neighbors) == 2:                   # 길이 두개라면 질문한다.
                p0 = self.get_node(self.nearest).point  # 중앙을 파악한다.
                p1 = self.get_node(neighbors[0]).point
                p2 = self.get_node(neighbors[1]).point

                th1 = math.atan2(p1.y-p0.y, p1.x-p0.x)
                th2 = math.atan2(p2.y-p0.y, p2.x-p0.x)
                dth = (th2 - th1 + math.pi)%(2*math.pi) - math.pi
                th = th1 + dth/2

                # rospy.loginfo([th1*180/math.pi, th2*180/math.pi, th*180/math.pi])

                self.head_target(self.nearest, th)




                # dx = pose.position.x - self.pose.position.x         # 방향을 설정한다.
                # dy = pose.position.y - self.pose.position.y
                #
                # get_node
                # th1 =
                # dth1 =


                # 질문
                # answer = self.get_motorimagery((neighbors[0], neighbors[1])).id
                # self.send_target(neighbors[0])
            else:
                rospy.loginfo("갈림길이 너무 많은데요?")


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

    def send_target(self, id):
        """이동로봇에게 목표자세를 전송한다"""
        pose = Pose()
        pose.position = self.get_node(id).point             # 위치를 설정한다.

        dx = pose.position.x - self.pose.position.x         # 방향을 설정한다.
        dy = pose.position.y - self.pose.position.y
        th = math.atan2(dy, dx)
        q = tf.transformations.quaternion_from_euler(0, 0, th)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        # self.publisher_lighter.publish(pose.position)   # 화면에 출력한다.

        self.publisher_target.publish(pose)             # 목표를 발행한다.
        self.history[1] = self.history[0]               # 이동했음을 기록한다.
        self.history[0] = id

    def head_target(self, id, th):
        """로봇이 특정 각도를 바라보도록 한다"""
        pose = Pose()
        pose.position = self.get_node(id).point             # 위치를 설정한다.
        q = tf.transformations.quaternion_from_euler(0, 0, th)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        # self.publisher_flicker.publish(pose.position)   # 화면에 출력한다.

        self.publisher_target.publish(pose)             # 목표를 발행한다.

    def explosion(self, event):
        """이동로봇의 임무계획시점을 판단한다"""
        need_plan = False       # 노드에 도달하여 대기중이라면 다음 움직임을 계획한다.
        try:

            # rospy.loginfo([self.state, self.move, self.history[0], self.nearest])

            if (self.state == 0)&\
               (self.move == 0)&\
               (self.history[0] == self.nearest):
               need_plan = True
        except:
            return

        if need_plan:
            self.state = 1
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

    def percussion(self, data):
        """트리거가 발생하면 이동목표를 갱신한다"""
        # if self.state == 0:
        #     self.state = 2
        #     rospy.Timer(rospy.Duration(rospy.get_param('~planning_cycle', 0.5)), self.planning, oneshot=True)

        if self.state == -1:            # 대기중인 로봇을 깨운다.
            self.state = 1
            rospy.Timer(rospy.Duration(rospy.get_param('~planning_cycle', 0.5)), self.planning, oneshot=True)

        # else:
        #     rospy.loginfo("잠깐만요(state=%d)"%self.state)

    def update_state(self, data):
        """로봇의 상태를 갱신한다"""
        self.move = data.data

    def update_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.pose = data                                                # 현재 자세를 갱신한다.
        try:
            self.nearest = self.get_nearest(self.pose.position).id   # 가장 가까운 노드를 갱신한다.
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
