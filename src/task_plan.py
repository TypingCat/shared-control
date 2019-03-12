#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import math
import copy
import tf
import actionlib
import termios, sys, select, tty

from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Int32, Header

from shared_control.msg import MID, MotorImageryResult, EyeblinkResult
from shared_control.srv import Nearest, Neighbors, Node, MotorImagery
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

C_RED   = "\033[31m"
C_GREEN = "\033[32m"
C_YELLO = "\033[33m"
C_END   = "\033[0m"

S_SLEEP = 0
S_DIRECT = 1
S_INDIRECT_WAIT = 2
S_INDIRECT_BUSY = 3

M_LEFT = 1
M_FORWARD = 2
M_RIGHT = 3
M_STOP = 4
M_BACKWARD = 5


class TaskPlan:
    """로봇의 구체적인 임무를 결정한다"""
    def __init__(self):
        # 파라미터 설정
        self.spin_cycle = rospy.Duration(rospy.get_param('~spin_cycle', 0.1))
        self.plan_cycle = rospy.Duration(rospy.get_param('~plan_cycle', 0.5))
        self.node_radius = rospy.get_param('~node_radius', 2.0)
        self.robot_vel_lin = rospy.get_param('~robot_vel_lin', 0.26)    # 최대 0.26
        self.robot_vel_ang = rospy.get_param('~robot_vel_ang', 0.50)    # 최대 1.82

        # 서비스 확인
        print(C_YELLO + '\rTask planner, GVG 서비스 확인중...' + C_END)
        self.eyeblink = rospy.get_time()
        rospy.wait_for_service('gvg/nearest')
        rospy.wait_for_service('gvg/neighbors')
        rospy.wait_for_service('gvg/node')
        self.get_nearest = rospy.ServiceProxy('gvg/nearest', Nearest)
        self.get_neighbors = rospy.ServiceProxy('gvg/neighbors', Neighbors)
        self.get_node = rospy.ServiceProxy('gvg/node', Node)
        print(C_YELLO + '\rTask planner, GVG 서비스 확인 완료' + C_END)

        print(C_YELLO + '\rTask planner, 자율주행 서비스 확인중...' + C_END)
        self.prev_goal = Point()
        self.move_result = GoalStatus()
        self.move_result.status = 3
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.Subscriber('interf/eyeblink_result', EyeblinkResult, self.percussion)
        rospy.Subscriber('robot/pose', PoseWithCovarianceStamped, self.update_robot_pose)
        rospy.Subscriber('move_base/result', MoveBaseActionResult, self.update_move_result)
        # self.publisher_target = rospy.Publisher('robot/target', Pose, queue_size=1)
        # self.publisher_douser = rospy.Publisher('visual/douser', Int32, queue_size=1)
        # self.publisher_MID_L = rospy.Publisher('visual/MID_L', MID, queue_size=1)
        # self.publisher_MID_R = rospy.Publisher('visual/MID_R', MID, queue_size=1)
        # self.publisher_MID_confirm = rospy.Publisher('visual/MID_confirm', MID, queue_size=1)
        self.robot_state = S_SLEEP
        self.mount_gvg()                                            # 이동을 시작한다.
        print(C_YELLO + '\rTask planner, 자율주행 서비스 확인 완료' + C_END)

        print(C_YELLO + '\rTask planner, BCI 서비스 확인중...' + C_END)
        rospy.wait_for_service('interf/motorimagery')
        self.get_motorimagery = rospy.ServiceProxy('interf/motorimagery', MotorImagery)
        print(C_YELLO + '\rTask planner, BCI 서비스 확인 완료' + C_END)

        # 초기화
        rospy.Timer(self.plan_cycle, self.explosion)
        self.publisher_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        print(C_GREEN + '\rTask planner, 초기화 완료\n' + C_END)

    def mount_gvg(self):
        """이동로봇을 GVG 위로 이동시킨다"""
        # 가장 가까운 노드를 성공할 때까지 검색한다.
        nearest = -1
        while nearest == -1:
            try:
                nearest = self.get_nearest(self.robot_pose.position).id
            except:
                rospy.sleep(self.plan_cycle)

        # 가장 가까운 노드로 이동한다.
        self.move_to(nearest)
        while self.move_result.goal_id.stamp.nsecs == 0:
            rospy.sleep(self.plan_cycle)

        # 현재상태를 기록한다.
        self.departure_node = nearest
        self.destination_node = nearest

    def percussion(self, data):
        """획득한 데이터를 장전-격발한다"""
        if data.num == 3:                   # 상태 반전(휴면<->작동)
            if self.robot_state == S_SLEEP:
                print('\rTask planner/percussion, 상태전환: ' + C_YELLO + '활성' + C_END)
                self.robot_state = S_INDIRECT_WAIT
            else:
                print('\rTask planner/percussion, 상태전환: ' + C_YELLO + '휴면' + C_END)
                self.robot_state = S_SLEEP

    def explosion(self, event):
        """로봇에 과부하를 걸어 폭발시킨다"""
        if not self.robot_state == S_INDIRECT_WAIT:
            return

        # 목적지 도달여부를 확인한다.
        des_node_pos = self.get_node(self.destination_node).point
        des_node_dist = math.sqrt((self.robot_pose.position.x - des_node_pos.x)**2
                                 +(self.robot_pose.position.y - des_node_pos.y)**2)
        # if not self.get_nearest(self.robot_pose.position).id == self.destination_node:
        #     return
        if des_node_dist > self.node_radius:
            return

        # 목표도달여부를 확인한다.
        # if (self.robot_state == S_INDIRECT_BUSY) and (self.move_result.status == 3):
        #     self.robot_state = S_INDIRECT_WAIT
        # 실행조건을 확인한다.
        # if not self.robot_state == S_INDIRECT_WAIT:
        #     return
        # if self.robot_state == S_INDIRECT_WAIT:
        #     self.robot_state = S_INDIRECT_BUSY
        # self.move_result.status == 3

        # 선택지를 확인한다.
        des_node_neighbors = list(self.get_neighbors(self.destination_node).ids)
        choice = des_node_neighbors
        try:
            choice.remove(self.departure_node)
        except: pass

        # 선택지가 없다면 휴면상태로 전환한다.
        if len(choice) == 0:
            print('\rTask planner/explosion, 상태전환: ' + C_YELLO + '휴면' + C_END)
            self.robot_state = S_SLEEP
            self.departure_node = self.destination_node
            self.destination_node = self.destination_node

        # 선택지가 하나라면 바로 이동한다.
        elif len(choice) == 1:
            print('\rTask planner/explosion, 다음 노드로 이동')
            self.departure_node = self.destination_node
            self.destination_node = choice[0]
            self.move_to(choice[0])

        # 선택지가 둘 이상이라면 motorimagery를 요청한다.
        else:
            print('\rTask planner/explosion, Motorimagery 요청')
            self.robot_state = S_INDIRECT_BUSY
            cue = Header()
            cue.stamp = rospy.Time.now()
            mi = self.get_motorimagery(cue)

            # 획득한 명령에 따라 목적지를 설정한다.
            print('\rTask planner/explosion, Motorimagery 획득:'),
            if mi.dir == M_LEFT:
                print(C_YELLO + '좌' + C_END)
            elif mi.dir == M_RIGHT:
                print(C_YELLO + '우' + C_END)
            else:
                print(C_RED + '미구현' + C_END)
                self.robot_state = S_INDIRECT_WAIT
                return

            # 교차로의 각도를 계산한다.
            dep_node_pos = self.get_node(self.departure_node).point
            th_base = math.atan2(des_node_pos.y - dep_node_pos.y,
                                 des_node_pos.x - dep_node_pos.x)
            choice_th_abs = []
            choice_th_rel = []
            for c in choice:
                pos = self.get_node(c).point
                th = math.atan2(pos.y - des_node_pos.y,
                                pos.x - des_node_pos.x)
                choice_th_abs.append(th)
                choice_th_rel.append(self.round(th - th_base))

            # 교차로에 도달할 때까지 대기한다.
            while not self.move_result.status == 3:
                rospy.sleep(self.spin_cycle)
            print('\rTask planner/explosion, 교차로 도달')

            # 명령에 따라 회전한다.
            if mi.dir == M_LEFT:
                id = choice_th_rel.index(max(choice_th_rel))
            elif mi.dir == M_RIGHT:
                id = choice_th_rel.index(min(choice_th_rel))
            self.head_to(choice_th_abs[id])

            # 이동한다.
            print('\rTask planner/explosion, 다음 노드로 이동')
            self.departure_node = self.destination_node
            self.destination_node = choice[id]
            self.move_to(choice[id])

            self.robot_state = S_INDIRECT_WAIT


        # need_plan = False                           # 노드에 도달하면 다음 움직임을 계획한다.
        # try:                                        # 단 초기화가 완료될 때까지는 실행하지 않는다.
        #     if (self.state == 0)&\
        #        (self.move == 0)&\
        #        (self.history[0] == self.history[1]):
        #        need_plan = True
        # except:
        #     return
        #
        # # print(self.state),
        #
        # if need_plan:                               # 계획을 시작한다.
        #     self.state = 1
        #
        #     self.publisher_douser.publish(self.state)
        #     rospy.Timer(rospy.Duration(rospy.get_param('~plan_cycle', 1.0)), self.planning, oneshot=True)
        #
        # elif (self.state == 2)&\
        #      (self.move == 0)&\
        #      (self.history[0] == self.history[1]):  # 목표에 도달했음을 기록한다.
        #     self.state = 0
        #     self.history[3] = self.history[2]
        #     self.history[2] = self.history[1]

        # 움직임이 종료될 때까지 대기한다.
        # if (not self.move_result.status == 3)
        # self.robot_state = S_INDIRECT_WAIT


    # def planning(self, event):
    #     """움직임을 계획한다"""
    #     neighbors = list(self.get_neighbors(self.history[2]).ids)
    #     try:                                    # 지나온 노드를 제외한 이웃을 검색한다.
    #         neighbors.remove(self.history[3])
    #     except: pass
    #
    #     if len(neighbors) == 0:                 # 막다른 길일 경우,
    #         print('대기합니다.')                  # 휴면상태로 전환한다.
    #         self.state = -1
    #
    #     elif len(neighbors) == 1:               # 길이 하나밖에 없다면,
    #         print('이동합니다.')                  # 질문할 필요 없이 이동한다.
    #         self.move_to(neighbors[0])
    #
    #     elif len(neighbors) == 2:               # 길이 두개라면,
    #         print('방향을 선택해 주세요.')          # MI로 질문한다.
    #
    #         p0 = self.get_node(self.history[1]).point
    #         p1 = self.get_node(neighbors[0]).point
    #         p2 = self.get_node(neighbors[1]).point
    #         dth, th1, th2 = self.head_target(p0, p1, p2)
    #         if dth > 0:
    #             neighbors[0], neighbors[1] = neighbors[1], neighbors[0]
    #             th1, th2 = th2, th1
    #
    #         left = MID()                        # 선택지 마커를 출력한다.
    #         left.point = p0
    #         left.th = th1
    #         self.publisher_MID_L.publish(left)
    #         right = MID()
    #         right.point = p0
    #         right.th = th2
    #         self.publisher_MID_R.publish(right)
    #
    #         answer = self.get_motorimagery((neighbors[0], neighbors[1])).id
    #         if answer == neighbors[0]:          # MI 서비스를 요청한다.
    #             self.prearrangement = [0, neighbors[0], neighbors[1], self.history[3]]
    #         else:
    #             self.prearrangement = [0, neighbors[1], neighbors[0], self.history[3]]
    #             th1, th2 = th2, th1
    #
    #         direction = MID()                   # 결과를 마커로 출력한다.
    #         direction.point = p0
    #         direction.th = th1
    #         self.publisher_MID_confirm.publish(direction)
    #
    #         rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.go_around, oneshot=True)      # 행동을 취한다.
    #
    #     else:
    #         self.state = -1
    #         rospy.loginfo('갈림길이... 너무 많은데요?')
    #
    # def go_around(self, event):
    #     """이동한다"""
    #     print('이동합니다.')
    #     self.move_to(self.prearrangement[1])
    #
    #     now = rospy.get_time()
    #     while self.state == 2:          # 목표에 도달하기 전에,
    #         if self.eyeblink > now:     # eyeblink가 들어오면 목표를 변경한다.
    #             self.eyeblink = now
    #             self.prearrangement[0] = self.prearrangement[0] + 1
    #             self.move_to(self.prearrangement[self.prearrangement[0]%3+1])
    #
    #             print('목표를 변경합니다.')
    #             self.publisher_douser.publish(self.state)
    #
    #         rospy.sleep(rospy.get_param('~spin_cycle', 0.1))

    def round(self, th):
        return ((th + math.pi) % (2 * math.pi)) - math.pi

    def move_to(self, id, force=False):
        """로봇을 해당노드로 이동시킨다"""
        # 아직 이전목표에 도달하지 않았다면 새로운 명령은 무시한다.
        if (not self.move_result.status == 3) and (force == False):
            return
        self.move_result.status = 0

        # 이동목표를 설정한다.
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'                        # 좌표계
        goal.target_pose.pose.position = self.get_node(id).point        # 자세
        dx = goal.target_pose.pose.position.x - self.robot_pose.position.x
        dy = goal.target_pose.pose.position.y - self.robot_pose.position.y
        th = math.atan2(dy, dx)
        q = tf.transformations.quaternion_from_euler(0, 0, th)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        # 이동명령을 내린다.
        if force == True:
            self.client.cancel_goal()
            self.client.wait_for_server()
        self.client.send_goal(goal)

    def head_to(self, th_target):
        """로봇이 해당각도만큼 회전한다"""
        # 목표에 도달할 때까지 회전한다.
        dth = 1
        while abs(dth) > 0.1:
            th = tf.transformations.euler_from_quaternion(
                [self.robot_pose.orientation.x, self.robot_pose.orientation.y,
                 self.robot_pose.orientation.z, self.robot_pose.orientation.w])[2]
            dth = self.round(th_target - th)
            vel = Twist()
            vel.angular.z = dth * self.robot_vel_ang
            self.publisher_cmd_vel.publish(vel)
            rospy.sleep(self.spin_cycle)

        # 정지한다.
        vel = Twist()
        self.publisher_cmd_vel.publish(vel)

    def update_move_result(self, data):
        """로봇의 상태를 갱신한다"""
        self.move_result = data.status

    def update_robot_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.robot_pose = data.pose.pose


if __name__ == '__main__':
    rospy.init_node('task_planner')
    tp = TaskPlan()
    rospy.spin()
