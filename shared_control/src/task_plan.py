#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import math
import copy
import tf
import actionlib
import termios, sys, select, tty

from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Int32, Header, Float32, Time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

from move_base_msgs.msg import MoveBaseActionFeedback

from shared_control.msg import CmdAssist, RobotMotion, NavCue
from shared_control.srv import Nearest, Neighbors, Node, Nav2Cmd

from reserved_words import *


class TaskPlan:
    def __init__(self):
        self.ang_fwd_threshold = rospy.get_param('~ang_fwd_threshold', 0.5)
        self.ang_vel_intersection = rospy.get_param('~ang_vel_intersection', 1.82)
        self.dist_alarm_1 = rospy.get_param('~dist_alarm_1', 5.0)
        self.dist_alarm_2 = rospy.get_param('~dist_alarm_2', 3.0)
        self.spin_cycle = rospy.Duration(rospy.get_param('~spin_cycle', 0.3))
        self.plan_cycle = rospy.Duration(rospy.get_param('~plan_cycle', 1.0))
        
        print(C_YELLO + '\rTask planner, GVG 서비스 확인중...' + C_END)
        rospy.wait_for_service('gvg/nearest')
        rospy.wait_for_service('gvg/neighbors')
        rospy.wait_for_service('gvg/node')
        self.get_nearest = rospy.ServiceProxy('gvg/nearest', Nearest)
        self.get_neighbors = rospy.ServiceProxy('gvg/neighbors', Neighbors)
        self.get_node = rospy.ServiceProxy('gvg/node', Node)
        print(C_YELLO + '\rTask planner, GVG 서비스 확인 완료' + C_END)

        print(C_YELLO + '\rTask planner, 자율주행 서비스 확인중...' + C_END)
        self.publisher_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.publisher_robot_motion = rospy.Publisher('interf/robot/motion', RobotMotion, queue_size=1)
        self.publisher_nav_cue = rospy.Publisher('interf/nav_cue', NavCue, queue_size=1)

        print(C_YELLO + '\rTask planner, Action 서비스 확인중...' + C_END)
        self.move_result = GoalStatus()
        self.move_result.status = 3
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.percussion_time = rospy.get_time()
        
        print(C_YELLO + '\rTask planner, 토픽 구독중...' + C_END)
        rospy.Subscriber('interf/cmd/assist', CmdAssist, self.percussion)
        rospy.Subscriber('robot/pose', PoseWithCovarianceStamped, self.update_robot_pose)
        rospy.Subscriber('move_base/result', MoveBaseActionResult, self.update_move_result)
        
        self.time_start = rospy.Time.now()
        rospy.Subscriber('time/start', Time, self.update_time)

        print(C_YELLO + '\rTask planner, 초기자세로 이동중...' + C_END)
        self.robot_state = S_INDIRECT_WAIT
        self.mount_gvg()
        print(C_GREEN + '\rTask planner, 자율주행 서비스 초기화 완료' + C_END)
        
        print(C_YELLO + '\rTask planner, BCI 서비스 확인중...' + C_END)
        rospy.wait_for_service('interf/nav2cmd')
        self.get_cmd = rospy.ServiceProxy('interf/nav2cmd', Nav2Cmd)
        print(C_YELLO + '\rTask planner, BCI 서비스 확인 완료' + C_END)
        
        print(C_YELLO + '\rTask planner, 기동중...' + C_END)
        rospy.Timer(self.plan_cycle, self.explosion)
        print(C_GREEN + '\rTask planner, 초기화 완료\n' + C_END)

    def mount_gvg(self):
        nearest = -1
        while nearest == -1:
            try:
                nearest = self.get_nearest(self.robot_pose.position).id
            except:
                rospy.sleep(self.plan_cycle)
        
        nearest_pos = self.get_node(nearest).point        
        while self.move_result.status == 0:
            rospy.sleep(self.plan_cycle)

        neighbors = list(self.get_neighbors(nearest).ids)
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
        if data.num == 3:
            print('\r%6.1f[s]: Task planner, 명령('%(rospy.Time.now() - self.time_start).to_sec() + C_YELLO + '3' + C_END + ') 획득')
            self.client.cancel_goal()
            self.client.wait_for_server()
            vel = Twist()
            self.publisher_cmd_vel.publish(vel)
            self.move_result.status = 3
            self.publisher_robot_motion.publish(header=self.get_header(), motion=M_STOP)
            self.robot_state = S_SLEEP
        elif data.num == 2:
            print('\r%6.1f[s]: Task planner, 명령('%(rospy.Time.now() - self.time_start).to_sec() + C_YELLO + '2' + C_END + ') 획득')
            self.percussion_time = rospy.get_time()
            if self.robot_state == S_SLEEP:
                self.robot_state = S_INDIRECT_WAIT

    def explosion(self, event):
        if (self.robot_state == S_INDIRECT_BUSY) or (self.robot_state == S_SLEEP):
            return
        des_node_pos = self.get_node(self.destination_node).point
        des_node_dist = math.sqrt((self.robot_pose.position.x - des_node_pos.x)**2
                                + (self.robot_pose.position.y - des_node_pos.y)**2)

        while des_node_dist > self.dist_alarm_1:
            des_node_pos = self.get_node(self.destination_node).point
            des_node_dist = math.sqrt((self.robot_pose.position.x - des_node_pos.x)**2
                                    + (self.robot_pose.position.y - des_node_pos.y)**2)
            rospy.sleep(self.spin_cycle)
        self.publisher_nav_cue.publish(self.check_intersection())

        while des_node_dist > self.dist_alarm_2:
            des_node_pos = self.get_node(self.destination_node).point
            des_node_dist = math.sqrt((self.robot_pose.position.x - des_node_pos.x)**2
                                    + (self.robot_pose.position.y - des_node_pos.y)**2)
            rospy.sleep(self.spin_cycle)

        des_node_neighbors = list(self.get_neighbors(self.destination_node).ids)
        choice = copy.copy(des_node_neighbors)
        try:
            choice.remove(self.departure_node)
        except: pass

        if len(choice) == 0:    
            while not self.move_result.status == 3:
                rospy.sleep(self.spin_cycle)
            print('\r%6.1f[s]: Task planner, 상태전환: '%(rospy.Time.now() - self.time_start).to_sec() + C_YELLO + '휴면' + C_END)
            self.departure_node = self.destination_node
            self.robot_state = S_SLEEP
            self.publisher_robot_motion.publish(header=self.get_header(), motion=M_STOP)
            return

        elif len(choice) == 1:
            print('\r%6.1f[s]: Task planner, 다음 노드로 이동'%(rospy.Time.now() - self.time_start).to_sec())
            self.departure_node = self.destination_node
            self.destination_node = choice[0]
            self.move_to(choice[0])
            self.robot_state = S_INDIRECT_WAIT
            self.publisher_robot_motion.publish(header=self.get_header(), motion=M_MOVE)
            return

        else:
            print('\r%6.1f[s]: Task planner, 명령 요청'%(rospy.Time.now() - self.time_start).to_sec())
            self.robot_state = S_INDIRECT_BUSY
            self.publisher_robot_motion.publish(header=self.get_header(), motion=M_CUE)
            cue = self.check_intersection()
            cmd = self.get_cmd(header=cue.header,
                               dist=cue.dist,
                               right=cue.right,
                               left=cue.left,
                               forward=cue.forward,
                               backward=cue.backward)
            if cmd.dir == M_FORWARD:
                print('\r%6.1f[s]: Task planner, 명령('%(rospy.Time.now() - self.time_start).to_sec() + C_YELLO + '앞' + C_END + ') 획득')
                self.publisher_robot_motion.publish(header=self.get_header(), motion=M_FORWARD)
            elif cmd.dir == M_LEFT:
                print('\r%6.1f[s]: Task planner, 명령('%(rospy.Time.now() - self.time_start).to_sec() + C_YELLO + '좌' + C_END + ') 획득')
                self.publisher_robot_motion.publish(header=self.get_header(), motion=M_LEFT)
            elif cmd.dir == M_RIGHT:
                print('\r%6.1f[s]: Task planner, 명령('%(rospy.Time.now() - self.time_start).to_sec() + C_YELLO + '우' + C_END + ') 획득')
                self.publisher_robot_motion.publish(header=self.get_header(), motion=M_RIGHT)
            while not self.move_result.status == 3:
                rospy.sleep(self.spin_cycle)

            if cmd.dir == M_FORWARD:
                pass
            elif cmd.dir == M_LEFT:
                self.turn(1)
            elif cmd.dir == M_RIGHT:
                self.turn(-1)
            else:
                self.robot_state = S_INDIRECT_WAIT
                return

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

            print('\r%6.1f[s]: Task planner, 다음 노드로 이동'%(rospy.Time.now() - self.time_start).to_sec())
            self.publisher_robot_motion.publish(header=self.get_header(), motion=M_MOVE)
            self.move_to(des_node_neighbors[id])
            self.departure_node = self.destination_node
            self.destination_node = des_node_neighbors[id]
            self.robot_state = S_INDIRECT_WAIT

    def round(self, th):
        return ((th + math.pi) % (2 * math.pi)) - math.pi

    def turn(self, sign):
        vel = Twist()
        vel.angular.z = sign * self.ang_vel_intersection
        self.publisher_cmd_vel.publish(vel)
        if sign > 0:
            self.publisher_robot_motion.publish(header=self.get_header(), motion=M_LEFT)
        else:
            self.publisher_robot_motion.publish(header=self.get_header(), motion=M_RIGHT)

        t = rospy.get_time()
        while self.percussion_time < t:
            self.publisher_cmd_vel.publish(vel)
            rospy.sleep(self.spin_cycle)
        vel.angular.z = 0
        self.publisher_cmd_vel.publish(vel)
        rospy.sleep(self.spin_cycle)

    def move_to(self, id, force=False):
        if (not self.move_result.status == 3) and (force == False):
            return
        self.move_result.status = 0
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

        if force == True:
            self.client.cancel_goal()
            self.client.wait_for_server()
        self.client.send_goal(goal)

    def check_intersection(self):
        des_node_neighbors = list(self.get_neighbors(self.destination_node).ids)
        choice = copy.copy(des_node_neighbors)
        try:
            choice.remove(self.departure_node)
        except: pass
        dep_pos = self.get_node(self.departure_node).point
        des_pos = self.get_node(self.destination_node).point
        th = math.atan2(des_pos.y - dep_pos.y,
                        des_pos.x - dep_pos.x)
        choice_pos = [self.get_node(node).point for node in choice]
        choice_th = [self.round(math.atan2(pos.y - des_pos.y, pos.x - des_pos.x) - th) for pos in choice_pos]

        cue = NavCue()
        cue.header = self.get_header()
        cue.dist = math.sqrt((self.robot_pose.position.x - des_pos.x)**2
                           + (self.robot_pose.position.y - des_pos.y)**2)

        cue.right = 0
        cue.left = 0
        cue.forward = 0
        cue.backward = 1
        choice_th_forward = [0, 3.14]
        for i, th in enumerate(choice_th):
            if abs(th) < abs(choice_th_forward[1]):
                choice_th_forward[0] = i
                choice_th_forward[1] = th
        if abs(choice_th_forward[1]) < self.ang_fwd_threshold:
            cue.forward = 1
            choice_th.remove(choice_th_forward[1])
        for i, th in enumerate(choice_th):
            if th < 0:
                cue.right += 1
            else:
                cue.left += 1
        return cue

    def update_move_result(self, data):
        self.move_result = data.status

    def update_robot_pose(self, data):
        self.robot_pose = data.pose.pose

    def update_time(self, data):
        self.time_start = data.data

    def get_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        return header
        

if __name__ == '__main__':
    rospy.init_node('task_planner')
    tp = TaskPlan()
    rospy.spin()
