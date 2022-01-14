#!/usr/bin/env python
#-*-coding: utf-8-*-

import interface

import rospy
import pygame
import copy
import os
import math

from sys import stdout
from std_msgs.msg import Int32, Header, Int32MultiArray, Float32, Time
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Image, Joy, JointState
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point

from shared_control.msg import NavCue, CmdIntuit, CmdAssist, RobotMotion
from shared_control.srv import Nav2Cmd, Node
from reserved_words import *


class Interf(interface.Interface):
    def __init__(self):
        # 파라미터 설정
        self.lin_vel_joy = rospy.get_param('~lin_vel_joy', 0.69)
        self.ang_vel_joy = rospy.get_param('~ang_vel_joy', 3.67)
        self.camera = rospy.get_param('~camera', 'camera/color/image_raw')
        self.spin_cycle = rospy.Duration(rospy.get_param('~spin_cycle', 0.05))
        self.scale_arrow = rospy.get_param('~scale_arrow', 50)
        self.scale_cross = rospy.get_param('~scale_cross', 30)

        # 화면 초기화
        os.environ['SDL_VIDEO_WINDOW_POS'] = "0, 0"
        pygame.init()
        self.arrow_switch = False
        self.cross_switch = False
        self.monitor = pygame.display.Info()
        self.width = rospy.get_param('~width', int(0.48*self.monitor.current_w))
        self.height = rospy.get_param('~height', int(0.48*self.monitor.current_h))
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.mouse.set_visible(False)
        pygame.display.set_caption("Shared control interface")

        # 토픽 구독
        print(C_YELLO + '\rInterfacer, BCI 서비스 준비중...' + C_END)
        rospy.Subscriber(self.camera, Image, self.visualize)
        self.color = {'data': [(255, 223, 36),   # default
                               (255, 223, 36),   # M_RIGHT
                               (255, 223, 36),   # M_LEFT
                               (255, 223, 36),   # M_FORWARD
                               (255, 223, 36),
                               (255, 223, 36),
                               (255, 223, 36),
                               (134, 229, 127)],  # M_MOVE
                      'time': [rospy.get_time()]*8}

        # 출력 설정
        self.publisher_cmd_intuit = rospy.Publisher('interf/cmd/intuit', CmdIntuit, queue_size=1)
        self.publisher_cmd_assist = rospy.Publisher('interf/cmd/assist', CmdAssist, queue_size=1)
        self.publisher_nav_cue = rospy.Publisher('interf/nav_cue', NavCue, queue_size=1)
        self.publisher_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.publisher_cmd_joint = rospy.Publisher('cmd_joint', JointState, queue_size=1)

        # 토픽 구독
        self.cmd = CmdIntuit()
        self.switch_marker = [False, False, False]
        rospy.Subscriber('interf/cmd/intuit', CmdIntuit, self.update_cmd_intuit)
        rospy.Subscriber('interf/robot/motion', RobotMotion, self.update_marker_color)
        rospy.Subscriber('interf/nav_cue', NavCue, self.update_marker_visibility)
        rospy.Subscriber('joy', Joy, self.joystick)
        self.path = []
        rospy.Subscriber('robot/pose', PoseWithCovarianceStamped, self.update_robot_pose)

        # 서비스 시작
        self.publisher_time_start = rospy.Publisher('time/start', Time, queue_size=1)
        self.publisher_time_end = rospy.Publisher('time/end', Time, queue_size=1)
        self.time_start = rospy.Time.now()
        # self.the_timer = rospy.Timer(rospy.Duration(0.1), self.timer)
        self.path_publisher = rospy.Publisher('interf/path', MarkerArray, queue_size=1)
        self.path_visualizer = rospy.Timer(rospy.Duration(0.3), self.visualize_path)

        rospy.Service('interf/nav2cmd', Nav2Cmd, self.nav2cmd)
        self.key_watcher = rospy.Timer(self.spin_cycle, self.keyboard)
        print(C_YELLO + '\rInterfacer, BCI 서비스 시작' + C_END)
        print(C_GREEN + '\rInterfacer, 초기화 완료' + C_END)

    def visualize(self, data):
        """화면을 출력한다."""
        # 영상을 획득한다.
        cam = pygame.image.frombuffer(data.data, (data.width, data.height), 'RGB')
        img = pygame.transform.smoothscale(cam, (self.width, self.height))
        self.screen.blit(img, (0, 0))
        # 영상 위에 화살표 마커를 덧붙여 출력한다.
        if self.arrow_switch:
            self.draw_arrow(M_RIGHT, 0.94*self.width, 0.5*self.height)
            self.draw_arrow(M_LEFT, 0.06*self.width, 0.5*self.height)
            self.draw_arrow(M_FORWARD, 0.5*self.width, 0.1*self.height)
        if self.cross_switch:
            self.draw_cross(0.5*self.width, 0.5*self.height)
        pygame.display.flip()

    def draw_arrow(self, type, x, y):
        """화살표를 그린다."""
        if type == M_RIGHT:
            arr = [[1, 0], [0, 1], [0, 0.5], [-1, 0.5], [-1, -0.5], [0, -0.5], [0, -1]]
        elif type == M_LEFT:
            arr = [[1, 0.5], [0, 0.5], [0, 1], [-1, 0], [0, -1], [0, -0.5], [1, -0.5]]
        elif type == M_FORWARD:
            arr = [[1, 0], [0.5, 0], [0.5, 1], [-0.5, 1], [-0.5, 0], [-1, 0], [0, -1]]
        arr = [[self.scale_arrow*i+x, self.scale_arrow*j+y] for [i, j] in arr]
        pygame.draw.polygon(self.screen, (255, 223, 36), arr)


class Simple:
    def __init__(self):
        self.interf = Interf()

        # 파라미터 획득
        self.move_dist = rospy.get_param('~move_dist', 0.5)
        self.move_vel = rospy.get_param('~move_vel', 0.3)
        self.move_time = rospy.get_param('~move_time', 2.0)
        self.turn_vel = rospy.get_param('~turn_vel', 1.82)
        self.wait_1 = rospy.get_param('~wait_1', 1.0)
        self.wait_2 = rospy.get_param('~wait_2', 3.0)

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
        self.publisher_simple = rospy.Publisher('simple/motion', RobotMotion, queue_size=1)

        # 실행
        self.interf.arrow_switch = False
        self.interf.cross_switch = False
        print(C_GREEN + '\rSimple, 초기화 완료' + C_END)
        rospy.sleep(rospy.Duration(0.2))

        while True:
            self.task()
            rospy.sleep(rospy.Duration(0.2))

    def task(self):
        '''질문과 이동을 반복한다.'''

        print('\r휴면')
        self.publisher_simple.publish(header=self.get_header(), motion=1)
        now = rospy.get_time()
        while self.time_cmd < now:
            rospy.sleep(rospy.Duration(0.2))

        print('\r화살표')
        self.publisher_simple.publish( header=self.get_header(), motion=2)
        self.interf.arrow_switch = True
        now = rospy.get_time()
        while rospy.get_time() < now+self.wait_1:
            rospy.sleep(rospy.Duration(0.2))

        print('\r픽스에이션')
        self.publisher_simple.publish(header=self.get_header(), motion=3)
        self.interf.cross_switch = True
        now = rospy.get_time()
        while rospy.get_time() < now+self.wait_2:
            rospy.sleep(rospy.Duration(0.2))

        print('\rStop cue')
        self.publisher_simple.publish(header=self.get_header(), motion=4)
        rospy.sleep(rospy.Duration(0.2))

        print('\r로봇 이동')
        self.move()
        rospy.sleep(rospy.Duration(0.2))
        
        print('\r화살표와 픽스에이션 제거')
        self.publisher_simple.publish(header=self.get_header(), motion=6)
        self.interf.arrow_switch = False
        self.interf.cross_switch = False
        rospy.sleep(rospy.Duration(0.2))

    def move(self):
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

    def move_forward(self, vel, time):
        '''주어진 시간동안 전진한다.'''
        self.publisher_robot_motion.publish(
            header=self.get_header(),
            motion=M_FORWARD)
        self.publisher_simple.publish(header=self.get_header(), motion=5)

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
