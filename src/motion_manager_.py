#!/usr/bin/env python
# -*- coding: utf-8 -*-  # 한글주석 처리용
'''
    Copyright (c) 2018, Sang-Seok Yun(ssyun@silla.ac.kr), Hong-ku Eum(ehg2y@naver.com)
    Silla University
    All rights reserved.

    본 프로그램은 BCI 파일럿 프로젝트에서  주어지는 로봇의 goal을 구독하게 되면
    ROS 네비게이션의 목적지 설정(MoveBaseGoal())하여 이동하고
    이동 중에는 로봇의 위치(/robot/pose)와 움직임상태(/robot/state)를 발행하고
    최종적으로 목적지에 도달하면 도착여부(/robot/goal)를 발행하는
    알고리즘으로 구성된다.

'''

import rospy
import roslib
import math

# to control navigation module
import actionlib

from time import time
from numpy import array
from std_msgs.msg import Int16,Int32, Int64
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import  Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry


class MotionManager(object):
    def __init__(self, name):
        self._action_name = name
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started..." % self.nodename)

        '''### initialize varibles'''
        self.status = 0
        self.target_linear_vel = 0
        self.target_angular_vel = 0
        self.control_linear_vel = 0
        self.control_angular_vel = 0
        self.prev_cmd = 0
        self.bSTART_MR = False

        self.goal = MoveBaseGoal()
        self.bUserCMD = False
        self.rob_x = 0 #odom x
        self.rob_y = 0 #odom y
        self.rob_th_z=0
        self.rob_th_w=0
        self.odom_cnt=0
        self.rob_px= 0
        self.rob_py= 0
        self.rob_pth_z=0
        self.rob_pth_w=0
        self.nRobMove=0 #0:STOP, 1:MOVE, 2:ROTATION


        '''### get parameters '''
        self.rate = rospy.get_param('~rate',30)

        '''### set subscribers/publishers'''
        rospy.Subscriber('/robot/target', Pose, self.cb_user_goal)
        rospy.Subscriber('/odom',Odometry, self.cb_odometry)

        self.rob_state = rospy.Publisher('robot/state', Int32, queue_size=1)
        self.rob_goal = rospy.Publisher('robot/goal', String, queue_size=1)
        self.rob_pose = rospy.Publisher('robot/pose', Pose , queue_size=1)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        '''### set main-routine ###'''
        rospy.Timer(rospy.Duration(0.1), self.main_loop)


    def cb_user_goal(self, msg):
        rospy.loginfo("User CMD: (%.2f, %.2f, %.2f, %.2f)", msg.position.x, msg.position.y, msg.orientation.z, msg.orientation.w)
        self.goal=self.set_goal_pose(msg)
        self.bUserCMD=True

    def set_goal_pose(self, loc):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.header.stamp = rospy.Time.now()
        goal_pose.target_pose.pose.position.x = loc.position.x
        goal_pose.target_pose.pose.position.y = loc.position.y
        goal_pose.target_pose.pose.position.z = loc.position.z
        goal_pose.target_pose.pose.orientation.x = loc.orientation.x
        goal_pose.target_pose.pose.orientation.y = loc.orientation.y
        goal_pose.target_pose.pose.orientation.z = loc.orientation.z
        goal_pose.target_pose.pose.orientation.w = loc.orientation.w
        return goal_pose

    def cb_odometry(self,msg):
        nRobMove=0 # 0:STOP, 1:MOVE, 2:ROTATION
        self.rob_x = msg.pose.pose.position.x
        self.rob_y = msg.pose.pose.position.y
        self.rob_th_z=msg.pose.pose.orientation.z
        self.rob_th_w=msg.pose.pose.orientation.w

        if self.odom_cnt >= 5:
            # rospy.loginfo("%.3f - %.3f ", math.fabs(self.rob_x - self.rob_px), math.fabs(self.rob_y - self.rob_py))
            if math.fabs(self.rob_x - self.rob_px)>=0.001 or math.fabs(self.rob_y-self.rob_py) >=0.001:
                nRobMove = 1
            elif math.fabs(self.rob_th_z - self.rob_pth_z)>=0.001 or math.fabs(self.rob_th_w-self.rob_pth_w) >=0.001:
                nRobMove = 2
            else:
                nRobMove = 0

            if nRobMove==1 and not self.nRobMove==1:
                self.nRobMove= 1
                self.rob_state.publish(1)
            elif nRobMove==2 and not self.nRobMove==2:
                self.nRobMove= 2
                # self.rob_state.publish(2)
            elif nRobMove==0 and not self.nRobMove==0:
                self.nRobMove= 0
                self.rob_state.publish(0)

            self.rob_px = self.rob_x
            self.rob_py = self.rob_y
            self.rob_pth_z=self.rob_th_z
            self.rob_pth_w=self.rob_th_w
            self.odom_cnt = 0
            # 로봇 위치좌표 publish
            self.rob_pose.publish(msg.pose.pose)

        self.odom_cnt=self.odom_cnt+1


    def calculate_vel(self) :
        if self.target_linear_vel > self.control_linear_vel:
            self.control_linear_vel = min( self.target_linear_vel, self.control_linear_vel + (0.01/4.0) )
        else:
            self.control_linear_vel = self.target_linear_vel

        if self.target_angular_vel > self.control_angular_vel:
            self.control_angular_vel = min( self.target_angular_vel, self.control_angular_vel + (0.1/4.0) )
        else:
            self.control_angular_vel = self.target_angular_vel

        twist = Twist()
        twist.linear.x = self.target_linear_vel; twist.linear.y=0; twist.linear.z=0
        twist.angular.x=0; twist.angular.y=0; twist.angular.z=self.target_angular_vel
        self.pub_vel.publish(twist)



    def setGoal(self, target):
        self.mr_base.send_goal(target)
        wait = self.mr_base.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            rospy.loginfo("Reached_Goal: %s", wait)
            return self.mr_base.get_result()


    def stop(self):
        rospy.logwarn("STOP - by user_command")
        self.target_linear_vel   = 0
        self.control_linear_vel  = 0
        self.target_angular_vel  = 0
        self.control_angular_vel = 0
        self.calculate_vel()


        '''### 로봇 이동 제어부 ### '''
    def main_loop(self, msg):

        if not self.bSTART_MR:
            rospy.loginfo("move_base starts....")
            self.mr_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            self.mr_base.wait_for_server()
            self.bSTART_MR = True

        # Task_planner에서 입력되는 목표위치로 이동
        if self.bUserCMD==True:
            self.bUserCMD=False
            self.stop()
            retVal = self.setGoal(self.goal) # 마지막 이웃노드(우추하단?)로 골설정
            # rospy.logerr(retVal)
            # 골이 성공한 경우에만 publish
            self.rob_goal.publish("Reached")



if __name__ == '__main__':
    """ main """
    rospy.init_node("motion_manager")
    mrch = MotionManager(rospy.get_name())
    rospy.spin()
