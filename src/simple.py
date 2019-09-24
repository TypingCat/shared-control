#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy

from std_msgs.msg import Time, Header
from geometry_msgs.msg import Twist

from shared_control.msg import CmdAssist, RobotMotion, NavCue
from shared_control.srv import Nav2Cmd
from reserved_words import *


class Simple:
    def __init__(self):
        # 파라미터 획득
        self.move_dist = rospy.get_param('~move_dist', 0.5)
        self.move_vel = rospy.get_param('~move_vel', 0.3)
        self.move_time = rospy.get_param('~move_time', 2.0)
        self.turn_vel = rospy.get_param('~turn_vel', 1.82)

        # 변수 초기화
        self.time_start = rospy.Time.now()
        self.time_cmd = rospy.get_time()
        
        # 신호 획득
        self.get_cmd = rospy.ServiceProxy('interf/nav2cmd', Nav2Cmd)
        rospy.Subscriber('interf/cmd/assist', CmdAssist, self.update_cmd)
        rospy.Subscriber('time/start', Time, self.update_time)

        # 발행 설정
        rospy.wait_for_service('interf/nav2cmd')
        self.publisher_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.publisher_robot_motion = rospy.Publisher('interf/robot/motion', RobotMotion, queue_size=1)
        self.publisher_nav_cue = rospy.Publisher('interf/nav_cue', NavCue, queue_size=1)

        # 실행
        print(C_GREEN + '\rSimple, 초기화 완료' + C_END)
        while True:
            self.task()
            rospy.sleep(rospy.Duration(0.2))

    def task(self):
        '''질문과 이동을 반복한다.'''
        print('\r%6.1f[s]: Simple, 명령 요청'%(rospy.Time.now() - self.time_start).to_sec())
        cmd = self.get_cmd(
            header=self.get_header(),
            dist=self.move_dist,
            right=1,
            left=1,
            forward=1,
            backward=1
        )

        if cmd.dir==M_FORWARD:
            print('\r%6.1f[s]: Simple, 명령('%(rospy.Time.now() - self.time_start).to_sec() + C_YELLO + '앞' + C_END + ') 획득')
            print('\r%6.1f[s]: Simple, 다음 노드로 이동'%(rospy.Time.now() - self.time_start).to_sec())
            self.move_forward(self.move_vel, self.move_time)
        elif cmd.dir==M_BACKWARD:
            print('\r%6.1f[s]: Simple, 다음 노드로 이동'%(rospy.Time.now() - self.time_start).to_sec())
            self.move_forward(-self.move_vel, self.move_time)
        elif cmd.dir==M_LEFT:
            print('\r%6.1f[s]: Simple, 명령('%(rospy.Time.now() - self.time_start).to_sec() + C_YELLO + '좌' + C_END + ') 획득')
            self.turn(self.move_vel)
            print('\r%6.1f[s]: Simple, 다음 노드로 이동'%(rospy.Time.now() - self.time_start).to_sec())
            self.move_forward(self.move_vel, self.move_time)
        elif cmd.dir==M_RIGHT:
            print('\r%6.1f[s]: Simple, 명령('%(rospy.Time.now() - self.time_start).to_sec() + C_YELLO + '우' + C_END + ') 획득')
            self.turn(-self.move_vel)
            print('\r%6.1f[s]: Simple, 다음 노드로 이동'%(rospy.Time.now() - self.time_start).to_sec())
            self.move_forward(self.move_vel, self.move_time)
        else:
            self.robot_state = S_INDIRECT_WAIT
            return        

    def move_forward(self, vel, time):
        '''주어진 시간동안 전진한다.'''
        self.publisher_robot_motion.publish(
            header=self.get_header(),
            motion=M_FORWARD)

        v = Twist()
        v.linear.x = vel
        now = rospy.get_time()
        while rospy.get_time() < now+time:
            self.publisher_cmd_vel.publish(v)
            rospy.sleep(rospy.Duration(0.2))
        v.linear.x = 0
        self.publisher_cmd_vel.publish(v)
        rospy.sleep(rospy.Duration(0.2))

    def turn(self, vel):
        '''주어진 방향으로 회전한다.'''
        if vel > 0:
            self.publisher_robot_motion.publish(
                header=self.get_header(),
                motion=M_LEFT)
        else:
            self.publisher_robot_motion.publish(
                header=self.get_header(),
                motion=M_RIGHT)

        v = Twist()
        v.angular.z = vel
        now = rospy.get_time()
        while self.time_cmd < now:
            self.publisher_cmd_vel.publish(v)
            rospy.sleep(rospy.Duration(0.2))
        v.angular.z = 0
        self.publisher_cmd_vel.publish(v)
        rospy.sleep(rospy.Duration(0.2))
    
    def get_header(self):
        '''헤더를 생성한다.'''
        header = Header()
        header.stamp = rospy.Time.now()
        return header

    def update_cmd(self, data):
        '''이동시점 관련명령을 갱신한다.'''
        if data.num==3:
            print('\r%6.1f[s]: Simple, 명령('%(rospy.Time.now() - self.time_start).to_sec() + C_YELLO + '3' + C_END + ') 획득')
            vel = Twist()
            self.publisher_cmd_vel.publish(vel)
        elif data.num==2:
            print('\r%6.1f[s]: Simple, 명령('%(rospy.Time.now() - self.time_start).to_sec() + C_YELLO + '2' + C_END + ') 획득')
            self.time_cmd = rospy.get_time()

    def update_time(self, data):
        '''시작시각을 갱신한다.'''
        self.time_start = data.data


if __name__=='__main__':
    rospy.init_node('simple')
    s = Simple()
    rospy.spin()
