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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

from move_base_msgs.msg import MoveBaseActionFeedback

from shared_control.msg import MID, EyeblinkResult, RobotState
from shared_control.srv import Nearest, Neighbors, Node, Motorimagery
from reserved_words import *


# 로봇의 임무 계획
class TaskPlan:
    def __init__(self):
        """초기화"""
        ## 파라미터 설정
        self.spin_cycle = rospy.Duration(rospy.get_param('~spin_cycle', 0.1))
        self.plan_cycle = rospy.Duration(rospy.get_param('~plan_cycle', 0.5))
        self.node_radius = rospy.get_param('~node_radius', 2.0)
        self.robot_vel_lin = rospy.get_param('~robot_vel_lin', 0.6)
        self.robot_vel_ang = rospy.get_param('~robot_vel_ang', 1.82)
        ## GVG 서비스 확인
        print(C_YELLO + '\rTask planner, GVG 서비스 확인중...' + C_END)
        rospy.wait_for_service('gvg/nearest')
        rospy.wait_for_service('gvg/neighbors')
        rospy.wait_for_service('gvg/node')
        self.get_nearest = rospy.ServiceProxy('gvg/nearest', Nearest)
        self.get_neighbors = rospy.ServiceProxy('gvg/neighbors', Neighbors)
        self.get_node = rospy.ServiceProxy('gvg/node', Node)
        print(C_YELLO + '\rTask planner, GVG 서비스 확인 완료' + C_END)
        ## 자율주행 서비스 확인
        print(C_YELLO + '\rTask planner, 자율주행 서비스 확인중...' + C_END)
        self.publisher_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.publisher_robot_state = rospy.Publisher('interf/robot_state', RobotState, queue_size=1)
        self.prev_goal = Point()
        self.move_result = GoalStatus()
        self.move_result.status = 3
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.eyeblink_time = rospy.get_time()
        rospy.Subscriber('interf/eyeblink_result', EyeblinkResult, self.percussion)
        rospy.Subscriber('robot/pose', PoseWithCovarianceStamped, self.update_robot_pose)

        rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.update_move_feedback)
        rospy.Subscriber('move_base/status', GoalStatusArray, self.update_move_status)
        rospy.Subscriber('move_base/result', MoveBaseActionResult, self.update_move_result)

        self.robot_state = S_INDIRECT_WAIT
        self.mount_gvg()
        print(C_GREEN + '\rTask planner, 자율주행 서비스 초기화 완료' + C_END)
        ## BCI 서비스 확인
        print(C_YELLO + '\rTask planner, BCI 서비스 확인중...' + C_END)
        rospy.wait_for_service('interf/motorimagery')
        self.get_motorimagery = rospy.ServiceProxy('interf/motorimagery', Motorimagery)
        print(C_YELLO + '\rTask planner, BCI 서비스 확인 완료' + C_END)
        ## Task planner 작동 시작
        rospy.Timer(self.plan_cycle, self.explosion)
        print(C_GREEN + '\rTask planner, 초기화 완료\n' + C_END)

    def mount_gvg(self):
        """이동로봇을 GVG 노드에 마운트"""
        ## 로봇과 가장 가까운 노드 검색
        nearest = -1
        while nearest == -1:
            try:
                nearest = self.get_nearest(self.robot_pose.position).id
            except:
                rospy.sleep(self.plan_cycle)
        ## 로봇을 가장 가까운 노드로 이동
        self.move_to(nearest)
        while self.move_result.status == 0:
            rospy.sleep(self.plan_cycle)
        ## 로봇이 놓인 노드의 형태 확인
        neighbors = list(self.get_neighbors(nearest).ids)
        if len(neighbors) < 2:
            self.departure_node = nearest
        else:
            nearest_pos = self.get_node(nearest).point
            nearest_th = math.atan2(self.robot_pose.position.y - nearest_pos.y,
                                    self.robot_pose.position.x - nearest_pos.x)
            closest_th = 2*math.pi
            for node in neighbors:
                pos = self.get_node(node).point
                th = math.atan2(pos.y - nearest_pos.y,
                                pos.x - nearest_pos.x)
                th_ = abs(self.round(th - nearest_th))
                if th_ < closest_th:
                    closest_th = th_
                    self.departure_node = node
        self.destination_node = nearest

    def percussion(self, data):
        """획득한 데이터를 장전-격발"""
        ## Eyeblink 신호 확인
        if data.num == 3:
            print('\rTask planner, Eyeblink(' + C_YELLO + '3' + C_END + ') 획득')
        elif data.num == 2:
            print('\rTask planner, Eyeblink(' + C_YELLO + '2' + C_END + ') 획득')
            self.eyeblink_time = rospy.get_time()
            if self.robot_state == S_SLEEP:
                self.robot_state = S_INDIRECT_WAIT

    def explosion(self, event):
        """로봇에 과부하를 걸어 폭발"""
        ## 중복실행 방지
        if (self.robot_state == S_INDIRECT_BUSY) or (self.robot_state == S_SLEEP):
            return
        ## 로봇이 목적지에 접근했는지 확인
        dep_node_pos = self.get_node(self.departure_node).point
        dep_node_dist = math.sqrt((self.robot_pose.position.x - dep_node_pos.x)**2
                                 +(self.robot_pose.position.y - dep_node_pos.y)**2)
        des_node_pos = self.get_node(self.destination_node).point
        des_node_dist = math.sqrt((self.robot_pose.position.x - des_node_pos.x)**2
                                 +(self.robot_pose.position.y - des_node_pos.y)**2)
        if dep_node_dist < des_node_dist:
            return
        if des_node_dist > self.node_radius:
            return
        ## 선택지 확인
        des_node_neighbors = list(self.get_neighbors(self.destination_node).ids)
        choice = copy.copy(des_node_neighbors)
        try:
            choice.remove(self.departure_node)
        except: pass
        if len(choice) == 0:
            ### 선택지가 없다면, 말단노드라면 정지하고 휴면상태로 전환한다.
            while not self.move_result.status == 3:
                rospy.sleep(self.spin_cycle)
            print('\rTask planner, 상태전환: ' + C_YELLO + '휴면' + C_END)
            self.departure_node = self.destination_node
            self.robot_state = S_SLEEP
            ### 정지 알림
            state = RobotState()
            state.motion = M_STOP
            self.publisher_robot_state.publish(state)
            return
        elif len(choice) == 1:
            ### 선택지가 하나라면, 선택이 불필요하다면 바로 이동한다.
            print('\rTask planner, 다음 노드로 이동')
            self.departure_node = self.destination_node
            self.destination_node = choice[0]
            self.move_to(choice[0])
            self.robot_state = S_INDIRECT_WAIT
            ### 움직임 알림
            state = RobotState()
            state.motion = M_FORWARD
            self.publisher_robot_state.publish(state)
            return
        else:
            ### 선택지가 둘 이상이라면 motorimagery를 요청한다.
            print('\rTask planner, Motorimagery 요청')
            self.robot_state = S_INDIRECT_BUSY
            cue = Header()
            cue.stamp = rospy.Time.now()
            mi = self.get_motorimagery(cue)
            if mi.dir == M_FORWARD:
                print('\rTask planner, Motorimagery(' + C_YELLO + '앞' + C_END + ') 획득')
            elif mi.dir == M_LEFT:
                print('\rTask planner, Motorimagery(' + C_YELLO + '좌' + C_END + ') 획득')
            elif mi.dir == M_RIGHT:
                print('\rTask planner, Motorimagery(' + C_YELLO + '우' + C_END + ') 획득')
            ### 교차로에 도달할 때까지 대기한다.
            while not self.move_result.status == 3:
                rospy.sleep(self.spin_cycle)
            ### 획득한 답변에 따라 움직임을 결정한다.
            if mi.dir == M_FORWARD:
                pass
            elif mi.dir == M_LEFT:
                self.turn(1)
            elif mi.dir == M_RIGHT:
                self.turn(-1)
            else:
                self.robot_state = S_INDIRECT_WAIT
                return
            ### 정면의 경로를 확인한다.
            des_node_neighbors_th = []
            for c in des_node_neighbors:
                pos = self.get_node(c).point
                th = math.atan2(pos.y - des_node_pos.y,
                                pos.x - des_node_pos.x)
                des_node_neighbors_th.append(th)
            th_robot = tf.transformations.euler_from_quaternion(
                [self.robot_pose.orientation.x, self.robot_pose.orientation.y,
                 self.robot_pose.orientation.z, self.robot_pose.orientation.w])[2]
            des_node_neighbors_dth = [abs(self.round(th - th_robot)) for th in des_node_neighbors_th]
            id = des_node_neighbors_dth.index(min(des_node_neighbors_dth))
            ### 이동한다.
            print('\rTask planner, 다음 노드로 이동')
            self.move_to(des_node_neighbors[id])
            self.departure_node = self.destination_node
            self.destination_node = des_node_neighbors[id]
            self.robot_state = S_INDIRECT_WAIT

    def round(self, th):
        return ((th + math.pi) % (2 * math.pi)) - math.pi

    def turn(self, sign):
        """로봇이 해당방향으로 회전"""
        ## 회전
        vel = Twist()
        vel.angular.z = sign * self.robot_vel_ang
        self.publisher_cmd_vel.publish(vel)
        ## 회전방향 알림
        state = RobotState()
        if sign > 0:
            state.motion = M_LEFT
        else:
            state.motion = M_RIGHT
        self.publisher_robot_state.publish(state)
        ## Eyeblink가 들어오면 정지
        t = rospy.get_time()
        while self.eyeblink_time < t:
            self.publisher_cmd_vel.publish(vel)
            rospy.sleep(self.spin_cycle)
        vel.angular.z = 0
        self.publisher_cmd_vel.publish(vel)
        rospy.sleep(self.spin_cycle)

    def move_to(self, id, force=False):
        """로봇이 해당노드로 이동"""
        ## 새로운 명령 무시
        if (not self.move_result.status == 3) and (force == False):
            return
        self.move_result.status = 0
        ## 목표 설정
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position = self.get_node(id).point
        dx = goal.target_pose.pose.position.x - self.robot_pose.position.x
        dy = goal.target_pose.pose.position.y - self.robot_pose.position.y
        th = math.atan2(dy, dx)
        q = tf.transformations.quaternion_from_euler(0, 0, th)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        ## 이동
        if force == True:
            self.client.cancel_goal()
            self.client.wait_for_server()
        # if math.sqrt(dx**2 + dy**2) > self.node_radius:
        #     self.client.send_goal(goal)
        # else:
        #     self.move_result.status = 3
        self.client.send_goal(goal)
        ## 움직임 알림
        state = RobotState()
        state.motion = M_FORWARD
        self.publisher_robot_state.publish(state)

    def update_move_feedback(self, data):
        pass

    def update_move_status(self, data):
        pass

    def update_move_result(self, data):
        self.move_result = data.status

    def update_robot_pose(self, data):
        self.robot_pose = data.pose.pose


# 노드 생성
if __name__ == '__main__':
    rospy.init_node('task_planner')
    tp = TaskPlan()
    rospy.spin()
