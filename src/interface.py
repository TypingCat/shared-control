#!/usr/bin/env python
#-*-coding: utf-8-*-

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


class Interface:
    """사용자 입력을 관리한다"""

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
        self.the_timer = rospy.Timer(rospy.Duration(0.1), self.timer)
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
        if self.color['time'][M_CUE] > self.color['time'][M_MOVE]:
            self.draw_arrow(M_RIGHT, 0.94*self.width, 0.5*self.height)
            self.draw_arrow(M_LEFT, 0.06*self.width, 0.5*self.height)
            self.draw_arrow(M_FORWARD, 0.5*self.width, 0.1*self.height)
            self.draw_cross(0.5*self.width, 0.5*self.height)
        elif rospy.get_time() < self.color['time'][M_MOVE] + 3.:
            self.draw_arrow(M_RIGHT, 0.94*self.width, 0.5*self.height)
            self.draw_arrow(M_LEFT, 0.06*self.width, 0.5*self.height)
            self.draw_arrow(M_FORWARD, 0.5*self.width, 0.1*self.height)
        else:
            self.color['data'][M_RIGHT] = self.color['data'][0]
            self.color['data'][M_LEFT] = self.color['data'][0]
            self.color['data'][M_FORWARD] = self.color['data'][0]
            self.draw_arrow(M_RIGHT, 0.94*self.width, 0.5*self.height)
            self.draw_arrow(M_LEFT, 0.06*self.width, 0.5*self.height)
            self.draw_arrow(M_FORWARD, 0.5*self.width, 0.1*self.height)
        pygame.display.flip()

    def draw_arrow(self, type, x, y):
        """화살표를 그린다."""
        if type == M_RIGHT:
            if self.switch_marker[0] == False: return
            arr = [[1, 0], [0, 1], [0, 0.5], [-1, 0.5], [-1, -0.5], [0, -0.5], [0, -1]]
        elif type == M_LEFT:
            if self.switch_marker[1] == False: return
            arr = [[1, 0.5], [0, 0.5], [0, 1], [-1, 0], [0, -1], [0, -0.5], [1, -0.5]]
        elif type == M_FORWARD:
            if self.switch_marker[2] == False: return
            arr = [[1, 0], [0.5, 0], [0.5, 1], [-0.5, 1], [-0.5, 0], [-1, 0], [0, -1]]
        arr = [[self.scale_arrow*i+x, self.scale_arrow*j+y] for [i, j] in arr]
        pygame.draw.polygon(self.screen, self.color['data'][type], arr)

    def draw_cross(self, x, y):
        """십자가를 그린다"""
        cross = [[1, 0.1], [0.1, 0.1], [0.1, 1], [-0.1, 1], [-0.1, 0.1], [-1, 0.1], [-1, -0.1], [-0.1, -0.1], [-0.1, -1], [0.1, -1], [0.1, -0.1], [1, -0.1]]
        cross = [[self.scale_cross*i+x, self.scale_cross*j+y] for [i, j] in cross]
        pygame.draw.polygon(self.screen, self.color['data'][M_MOVE], cross)

    def nav2cmd(self, request):
        """Navigation cue를 발행하고 cmd를 받아온다."""
        # 신호를 발행한다.
        cue             = NavCue()
        cue.header      = copy.copy(request.header)
        cue.dist        = copy.copy(request.dist)
        cue.right       = copy.copy(request.right)
        cue.left        = copy.copy(request.left)
        cue.forward     = copy.copy(request.forward)
        cue.backward    = copy.copy(request.backward)
        self.publisher_nav_cue.publish(cue)

        # 명령을 획득한다.
        while self.cmd.header.stamp < cue.header.stamp:
            rospy.sleep(self.spin_cycle)
        return {'header': self.cmd.header,
                'dir': self.cmd.dir}

    def keyboard(self, event):
        """키보드 입력을 받아온다."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:       # 종료: ctrl+c
                self.key_watcher.shutdown()
                self.the_timer.shutdown()
                pygame.quit()
                rospy.sleep(rospy.Duration(1.0))
                rospy.signal_shutdown("Quit")
            elif event.type == pygame.KEYDOWN:  # 키 획득
                pressed = pygame.key.get_pressed()
                buttons = [pygame.key.name(k) for k, v in enumerate(pressed) if v]
                try:
                    buttons[0]
                except:
                    return

                if ((buttons[0] == 'c') and (len(buttons) > 2)) or (buttons[0] == 'escape'):   # 종료: ctrl+c
                    self.key_watcher.shutdown()
                    self.the_timer.shutdown()
                    pygame.quit()
                    rospy.sleep(rospy.Duration(1.0))
                    rospy.signal_shutdown("종료")
                elif buttons[0] == 'a':
                    self.publisher_cmd_intuit.publish(header=self.get_header(), dir=M_LEFT)
                elif buttons[0] == 'w':
                    self.publisher_cmd_intuit.publish(header=self.get_header(), dir=M_FORWARD)
                elif buttons[0] == 'd':
                    self.publisher_cmd_intuit.publish(header=self.get_header(), dir=M_RIGHT)
                elif buttons[0] == 's':
                    self.publisher_cmd_intuit.publish(header=self.get_header(), dir=M_STOP)
                elif buttons[0] == 'x':
                    self.publisher_cmd_intuit.publish(header=self.get_header(), dir=M_BACKWARD)

                if buttons[0] == '2':
                    self.publisher_cmd_assist.publish(header=self.get_header(), num=2)
                elif buttons[0] == '3':
                    self.publisher_cmd_assist.publish(header=self.get_header(), num=3)

                if buttons[0] == 'q':
                    self.time_start = rospy.Time.now()
                    self.publisher_time_start.publish(self.time_start)
                    self.path = []
                    print(C_YELLO + '\r%6.1f[s]: Interface, 시간 초기화'%(rospy.Time.now() - self.time_start).to_sec() + C_END)
                elif buttons[0] == 'e':
                    t = rospy.Time.now()
                    self.publisher_time_end.publish(t)
                    dist = 0.0
                    for i in range(len(self.path)-1): 
                        dist += math.sqrt((self.path[i+1][0] - self.path[i][0])**2 + (self.path[i+1][1] - self.path[i][1])**2)
                    print(C_GREEN + '\r%6.1f[s]: Interface, 이동거리 %3.1f[m]'%((t - self.time_start).to_sec(), dist) + C_END)

                if buttons[0] == 'j':   # Left
                    self.move_with(0, 1)
                elif buttons[0] == 'i': # Forward
                    self.move_with(1, 0)
                elif buttons[0] == 'l': # Right
                    self.move_with(0, -1)
                elif buttons[0] == ',': # Backward
                    self.move_with(-1, 0)
                elif buttons[0] == 'k': # Stop
                    self.move_with(0, 0)
                elif buttons[0] == 'u': # Forward-left
                    self.move_with(1, 1)
                elif buttons[0] == 'o': # Forward-right
                    self.move_with(1, -1)

                if buttons[0] == 'y':
                    self.head_to(0.5, 0.1)
                elif buttons[0] == 'h':
                    self.head_to(0.0, 0.1)
                elif buttons[0] == 'n':
                    self.head_to(-0.5, 0.1)

    def timer(self, event):
        elapsed = rospy.Time.now() - self.time_start
        stdout.write("\r%6.1f[s]: 현재 시각"%elapsed.to_sec())
        stdout.flush()

    def move_with(self, lin, ang):
        twist = Twist()
        twist.linear.x = self.lin_vel_joy * lin
        twist.angular.z = self.ang_vel_joy * ang
        self.publisher_cmd_vel.publish(twist)

    def head_to(self, pos, vel):
        joint = JointState()
        joint.position = [pos]
        joint.velocity = [vel]
        self.publisher_cmd_joint.publish(joint)

    def joystick(self, data):
        threshold = 0.2

        twist = Twist()
        twist.linear.x = self.lin_vel_joy * data.axes[1] if abs(data.axes[1]) > threshold else 0.0
        twist.angular.z = self.ang_vel_joy * data.axes[0] if abs(data.axes[0]) > threshold else 0.0
        self.publisher_cmd_vel.publish(twist)

        self.head_to(0.5*data.axes[4], 0.1)

        if data.buttons[11] == 1:
            self.publisher_cmd_intuit.publish(header=self.get_header(), dir=M_LEFT)
        if data.buttons[13] == 1:
            self.publisher_cmd_intuit.publish(header=self.get_header(), dir=M_FORWARD)
        if data.buttons[12] == 1:
            self.publisher_cmd_intuit.publish(header=self.get_header(), dir=M_RIGHT)
        if data.buttons[14] == 1:
            self.publisher_cmd_intuit.publish(header=self.get_header(), dir=M_BACKWARD)
        
        if data.buttons[1] == 1:    # B
            self.publisher_cmd_assist.publish(header=self.get_header(), num=2)
        elif data.buttons[2] == 1:  # X
            self.publisher_cmd_assist.publish(header=self.get_header(), num=3)

        if data.axes[2] == -1:
            self.time_start = rospy.Time.now()
            self.publisher_time_start.publish(self.time_start)
            self.path = []
            print(C_YELLO + '\r%6.1f[s]: Interface, 시간 초기화'%(rospy.Time.now() - self.time_start).to_sec() + C_END)
        if data.axes[5] == -1:
            t = rospy.Time.now()
            self.publisher_time_end.publish(t)
            dist = 0.0
            for i in range(len(self.path)-1): 
                dist += math.sqrt((self.path[i+1][0] - self.path[i][0])**2 + (self.path[i+1][1] - self.path[i][1])**2)
            print(C_GREEN + '\r%6.1f[s]: Interface, 이동거리 %3.1f[m]'%((t - self.time_start).to_sec(), dist) + C_END)

    def update_marker_color(self, data):
        self.color['time'][data.motion] = rospy.get_time()
        if (data.motion == M_RIGHT) or (data.motion == M_LEFT) or (data.motion == M_FORWARD):
            self.color['data'][data.motion] = (241, 95, 95)

    def update_marker_visibility(self, data):
        self.switch_marker[0] = True if data.right > 0 else False
        self.switch_marker[1] = True if data.left > 0 else False
        self.switch_marker[2] = True if data.forward > 0 else False

    def update_cmd_intuit(self, data):
        self.cmd.header = data.header
        self.cmd.dir = data.dir

    def get_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        return header

    def update_robot_pose(self, data):
        self.robot_pose = data.pose.pose
        self.path.append((self.robot_pose.position.x, self.robot_pose.position.y))

    def visualize_path(self, event):
        path_line = Marker()
        path_line.header.stamp = rospy.Time.now()
        path_line.header.frame_id = 'map'
        path_line.id = 0
        path_line.type = Marker.LINE_STRIP
        path_line.action = Marker.ADD
        path_line.scale.x = 0.3
        path_line.color.r = 0.0
        path_line.color.g = 1.0
        path_line.color.b = 0.0
        path_line.color.a = 0.3
        path_line.points = []
        for pos in self.path:
            p = Point()
            p.x = pos[0]
            p.y = pos[1]
            p.z = 0.01
            path_line.points.append(p)

        self.path_publisher.publish([path_line])

if __name__ == '__main__':
    rospy.init_node('interfacer')
    i = Interface()
    rospy.spin()
