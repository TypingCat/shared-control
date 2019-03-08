#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import math
import copy
import tf
import actionlib
import termios, sys, select, tty

from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Int32

from shared_control.msg import MID
from shared_control.srv import Nearest, Neighbors, Node, MotorImagery
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

C_RED   = "\033[31m"
C_GREEN = "\033[32m"
C_YELLO = "\033[33m"
C_END   = "\033[0m"


class TASK_PLAN:
    """로봇의 구체적인 임무를 결정한다"""
    def __init__(self):
        print(C_YELLO + 'Task planner, GVG 서비스 확인중...' + C_END)
        self.eyeblink = rospy.get_time()
        rospy.wait_for_service('gvg/nearest')
        rospy.wait_for_service('gvg/neighbors')
        rospy.wait_for_service('gvg/node')

        self.get_nearest = rospy.ServiceProxy('gvg/nearest', Nearest)
        self.get_neighbors = rospy.ServiceProxy('gvg/neighbors', Neighbors)
        self.get_node = rospy.ServiceProxy('gvg/node', Node)
        self.get_motorimagery = rospy.ServiceProxy('bci/motorimagery', MotorImagery)
        print(C_YELLO + 'Task planner, GVG 서비스 확인 완료' + C_END)

        print(C_YELLO + 'Task planner, 자율주행 서비스 확인중...' + C_END)
        self.prev_goal = Point()
        self.move_result = GoalStatus()
        self.move_result.status = 3
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # rospy.Subscriber('bci/eyeblink', Int32, self.percussion)
        # rospy.Subscriber('robot/state', Int32, self.update_state)
        rospy.Subscriber('robot/pose', PoseWithCovarianceStamped, self.update_pose)
        # rospy.Subscriber('move_base/status', GoalStatusArray, self.update_state)
        rospy.Subscriber('move_base/result', MoveBaseActionResult, self.update_move_result)

        # self.publisher_target = rospy.Publisher('robot/target', Pose, queue_size=1)
        # self.publisher_douser = rospy.Publisher('interface/douser', Int32, queue_size=1)
        # self.publisher_MID_L = rospy.Publisher('interface/MID_L', MID, queue_size=1)
        # self.publisher_MID_R = rospy.Publisher('interface/MID_R', MID, queue_size=1)
        # self.publisher_MID_confirm = rospy.Publisher('interface/MID_confirm', MID, queue_size=1)

        self.mount_gvg()                                            # 이동을 시작한다.
        print(C_YELLO + 'Task planner, 자율주행 서비스 확인 완료' + C_END)

        # print(C_YELLO + 'Task planner, BCI 서비스 확인중...' + C_END)
        # rospy.wait_for_service('bci/motorimagery')
        # print(C_YELLO + 'Task planner, BCI 서비스 확인 완료' + C_END)

        # rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.explosion)
        print(C_GREEN + 'Task planner, 초기화 완료' + C_END)

    #     self.publisher_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    #     self.key_setting = termios.tcgetattr(sys.stdin)
    #     self.key_watcher = rospy.Timer(rospy.Duration(rospy.get_param('~spin_cycle', 0.1)), self.keyboard)
    #
    # def get_key(self):
    #     """키보드 입력을 획득한다"""
    #     tty.setraw(sys.stdin.fileno())
    #     select.select([sys.stdin], [], [], 0)
    #     key = sys.stdin.read(1)
    #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.key_setting)
    #     return key
    #
    # def keyboard(self, event):
    #     """키보드 입력을 제어명령으로 변환한다"""
    #     cmd_vel = Twist()
    #     key = self.get_key()
    #     if key == '\x03':   # ctrl+c
    #         self.key_watcher.shutdown()
    #     elif key == 'a':    # left
    #         cmd_vel.linear.x = 0.
    #         cmd_vel.angular.z = 1.0
    #     elif key == 'w':    # forward
    #         cmd_vel.linear.x = 0.26
    #         cmd_vel.angular.z = 0.
    #     elif key == 'd':    # right
    #         cmd_vel.linear.x = 0.
    #         cmd_vel.angular.z = -1.0
    #     elif key == 's':    # stop
    #         cmd_vel.linear.x = 0.
    #         cmd_vel.angular.z = 0.
    #     elif key == 'x':    # backward
    #         cmd_vel.linear.x = -0.26
    #         cmd_vel.angular.z = 0.
    #
    #     self.publisher_cmd_vel.publish(cmd_vel)

    def mount_gvg(self):
        """이동로봇을 GVG 위로 이동시킨다"""
        nearest = -1
        while nearest == -1:    # 가장 가까운 노드를 성공할 때까지 검색한다.
            try:
                nearest = self.get_nearest(self.pose.position).id
            except:
                rospy.sleep(rospy.get_param('~spin_cycle', 0.1))

        # self.history = [nearest, nearest, nearest, nearest] # 목표, 현재, 최근, 이전

        # self.move = 0
        # while self.move == 0:
        self.move_to(nearest)   # 가장 가까운 노드로 이동한다.
        while self.move_result.goal_id.stamp.nsecs == 0:
            rospy.sleep(rospy.get_param('~plan_cycle', 1.0))

    # def percussion(self, data):
    #     """트리거가 발생하면 이동목표를 갱신한다"""
    #     if self.state == -1:                        # 휴면 상태인 로봇을 깨운다.
    #         self.state = 0
    #
    #         neighbors = list(self.get_neighbors(self.history[1]).ids)
    #         if len(neighbors) == 1:                 # 길이 하나밖에 없다면,
    #             print('이동합니다.')                  # 질문할 필요 없이 이동한다.
    #             self.move_to(neighbors[0])
    #
    #     else:                                       # 아니면 시간을 기록한다.
    #         self.eyeblink = rospy.get_time()
    #
    # def explosion(self, event):
    #     """이동로봇의 임무계획시점을 판단한다"""
    #     need_plan = False                           # 노드에 도달하면 다음 움직임을 계획한다.
    #     try:                                        # 단 초기화가 완료될 때까지는 실행하지 않는다.
    #         if (self.state == 0)&\
    #            (self.move == 0)&\
    #            (self.history[0] == self.history[1]):
    #            need_plan = True
    #     except:
    #         return
    #
    #     # print(self.state),
    #
    #     if need_plan:                               # 계획을 시작한다.
    #         self.state = 1
    #
    #         self.publisher_douser.publish(self.state)
    #         rospy.Timer(rospy.Duration(rospy.get_param('~plan_cycle', 1.0)), self.planning, oneshot=True)
    #
    #     elif (self.state == 2)&\
    #          (self.move == 0)&\
    #          (self.history[0] == self.history[1]):  # 목표에 도달했음을 기록한다.
    #         self.state = 0
    #         self.history[3] = self.history[2]
    #         self.history[2] = self.history[1]
    #
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

    def move_to(self, id, force=False):
        """로봇을 해당노드로 이동시킨다"""
        if (not self.move_result.status == 3) and (force == False):
            return

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'                        # 좌표계를 설정한다.
        goal.target_pose.pose.position = self.get_node(id).point        # 위치를 설정한다.
        # nearest = Point()
        # nearest = self.get_node(self.get_nearest(self.pose.position).id).point
        # rospy.loginfo(nearest)
        dx = goal.target_pose.pose.position.x - self.pose.position.x    # 방향을 설정한다.
        dy = goal.target_pose.pose.position.y - self.pose.position.y
        th = math.atan2(dy, dx)
        q = tf.transformations.quaternion_from_euler(0, 0, th)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        # print('({0}, {1}, {2}) vs ({3}, {4}, {5})'.format(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, th, self.prev_goal.x, self.prev_goal.y, self.prev_goal.z))

        # diff = abs(goal.target_pose.pose.position.x - self.prev_goal.x)\
        #      + abs(goal.target_pose.pose.position.y - self.prev_goal.y)\
        #      + abs(th - self.prev_goal.z)                               # 목표중복을 검토한다.
        # if diff > 0.1:
        #     self.prev_goal.x = goal.target_pose.pose.position.x         # 목표로 이동한다.
        #     self.prev_goal.y = goal.target_pose.pose.position.y
        #     self.prev_goal.z = th
        #     self.client.cancel_goal()
        #     self.client.wait_for_server()
        #     self.client.send_goal(goal)
        #     print(C_RED + '{}, 목표중복으로 취소'.format(id) + C_END)

        if force == True:
            self.client.cancel_goal()
            self.client.wait_for_server()
        self.client.send_goal(goal)

    def head_to(self, id, force=False):
        """로봇이 특정 각도를 바라보도록 한다"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'                    # 좌표계를 설정한다.
        goal.target_pose.pose.position.x = self.pose.position.x     # 위치를 설정한다.
        goal.target_pose.pose.position.y = self.pose.position.y
        p = Pose()
        p.position = self.get_node(id).point
        dx = p.position.x - self.pose.position.x                    # 방향을 설정한다.
        dy = p.position.y - self.pose.position.y
        th = math.atan2(dy, dx)
        q = tf.transformations.quaternion_from_euler(0, 0, th)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        diff = abs(goal.target_pose.pose.position.x                     # 중복성을 검토한다.
                 - self.goal_prev.target_pose.pose.position.x)\
             + abs(goal.target_pose.pose.position.y
                 - self.goal_prev.target_pose.pose.position.y)\
             + abs(goal.target_pose.pose.orientation.x
                 - self.goal_prev.target_pose.pose.orientation.x)\
             + abs(goal.target_pose.pose.orientation.y
                 - self.goal_prev.target_pose.pose.orientation.y)\
             + abs(goal.target_pose.pose.orientation.z
                 - self.goal_prev.target_pose.pose.orientation.z)\
             + abs(goal.target_pose.pose.orientation.w
                 - self.goal_prev.target_pose.pose.orientation.w)
        if diff < 0.1:
            return

        print(C_YELLO + '{}'.format(diff) + C_END)
        if diff > 0.1:
            self.goal_prev = copy.copy(goal)
            self.client.cancel_goal()                                   # 목표로 이동한다.
            self.client.wait_for_server()
            self.client.send_goal(goal)

    def update_move_result(self, data):
        """로봇의 상태를 갱신한다"""
        self.move_result = data.status
        # try:
        #     self.move_result = data.status_list[-1]
        # except: pass
    #     self.move = data.data

    def update_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.pose = data.pose.pose      # 현재 자세를 갱신한다.
        # try:                            # 가장 가까운 노드를 갱신한다.
        #     self.history[1] = self.get_nearest(self.pose.position).id
        # except: pass

if __name__ == '__main__':
    rospy.init_node('task_planner')
    tp = TASK_PLAN()
    rospy.spin()
