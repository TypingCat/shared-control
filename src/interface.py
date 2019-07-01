#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import pygame
import copy
import termios, sys, select, tty

from std_msgs.msg import Int32, Header
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Image

from shared_control.msg import MotorimageryCue, MotorimageryResult, EyeblinkResult, RobotState, PathState
from shared_control.srv import Motorimagery, Node
from reserved_words import *


class Interface:
    """사용자 입력을 관리한다"""

    def __init__(self):
        # 파라미터 설정
        self.spin_cycle = rospy.Duration(rospy.get_param('~spin_cycle', 0.1))

        # 화면 초기화
        pygame.init()
        pygame.display.set_caption("Shared control interface")
        rospy.Subscriber('camera/rgb/image_raw', Image, self.visualize)
        self.color = {'data': [(255, 223, 36),   # default
                               (255, 223, 36),   # M_RIGHT
                               (255, 223, 36),   # M_LEFT
                               (255, 223, 36),   # M_FORWARD
                               (255, 223, 36),
                               (255, 223, 36),
                               (255, 223, 36),
                               (134, 229, 127)],  # M_MOVE
                      'time': [rospy.get_time()]*8}

        # 입출력 설정
        print(C_YELLO + '\rInterfacer, BCI 서비스 준비중...' + C_END)
        self.publisher_motorimagery_cue = rospy.Publisher('interf/motorimagery_cue', MotorimageryCue, queue_size=1)
        self.motorimagery_header = Header()
        self.motorimagery_result = MotorimageryResult()
        rospy.Subscriber('interf/motorimagery_result', MotorimageryResult, self.update_motorimagery_result)
        rospy.Subscriber('interf/robot_state', RobotState, self.update_marker)
        rospy.Subscriber('interf/path_state', PathState, self.update_path)
        self.switch_marker = [False, False, False]

        self.publisher_motorimagery_result = rospy.Publisher('interf/motorimagery_result', MotorimageryResult, queue_size=1)
        rospy.Service('interf/motorimagery', Motorimagery, self.motorimagery)
        print(C_YELLO + '\rInterfacer, BCI 서비스 시작' + C_END)
        print(C_GREEN + '\rInterfacer, 초기화 완료' + C_END)

    def visualize(self, data):
        """화면을 출력한다"""
        # 화면 생성
        try:
            self.screen
        except:
            # self.screen = pygame.display.set_mode((data.width, data.height), pygame.FULLSCREEN)
            self.screen = pygame.display.set_mode((data.width, data.height))
            self.width = data.width
            self.height = data.height
        cam = pygame.image.frombuffer(data.data, (data.width, data.height), 'RGB')
        self.screen.blit(cam, (0, 0))
        # 방향 표현
        if self.color['time'][M_CUE] > self.color['time'][M_MOVE]:
            self.draw_arrow(M_RIGHT, 70, 0.94*self.width, 0.5*self.height)
            self.draw_arrow(M_LEFT, 70, 0.06*self.width, 0.5*self.height)
            self.draw_arrow(M_FORWARD, 70, 0.5*self.width, 0.1*self.height)
            self.draw_cross(50, 0.5*self.width, 0.5*self.height)
        elif rospy.get_time() < self.color['time'][M_MOVE] + 3.:
            self.draw_arrow(M_RIGHT, 70, 0.94*self.width, 0.5*self.height)
            self.draw_arrow(M_LEFT, 70, 0.06*self.width, 0.5*self.height)
            self.draw_arrow(M_FORWARD, 70, 0.5*self.width, 0.1*self.height)
        else:
            self.color['data'][M_RIGHT] = self.color['data'][0]
            self.color['data'][M_LEFT] = self.color['data'][0]
            self.color['data'][M_FORWARD] = self.color['data'][0]
        # 화면 출력
        pygame.display.flip()

    def draw_arrow(self, type, scale, x, y):
        """화살표를 그린다"""
        # 좌표 생성
        if type == M_RIGHT:
            if self.switch_marker[0] == False: return
            arr = [[1, 0], [0, 1], [0, 0.5], [-1, 0.5], [-1, -0.5], [0, -0.5], [0, -1]]
        elif type == M_LEFT:
            if self.switch_marker[1] == False: return
            arr = [[1, 0.5], [0, 0.5], [0, 1], [-1, 0], [0, -1], [0, -0.5], [1, -0.5]]
        elif type == M_FORWARD:
            if self.switch_marker[2] == False: return
            arr = [[1, 0], [0.5, 0], [0.5, 1], [-0.5, 1], [-0.5, 0], [-1, 0], [0, -1]]
        arr = [[scale*i+x, scale*j+y] for [i, j] in arr]
        # 출력
        pygame.draw.polygon(self.screen, self.color['data'][type], arr)

    def draw_cross(self, scale, x, y):
        """화살표를 그린다"""
        # 좌표 생성
        cross = [[1, 0.1], [0.1, 0.1], [0.1, 1], [-0.1, 1], [-0.1, 0.1], [-1, 0.1], [-1, -0.1], [-0.1, -0.1], [-0.1, -1], [0.1, -1], [0.1, -0.1], [1, -0.1]]
        cross = [[scale*i+x, scale*j+y] for [i, j] in cross]
        # 출력
        pygame.draw.polygon(self.screen, self.color['data'][M_MOVE], cross)

    def update_marker(self, data):
        """로봇의 상황으로 마커의 출력상태를 결정한다"""
        self.color['time'][data.motion] = rospy.get_time()
        if (data.motion == M_RIGHT) or (data.motion == M_LEFT) or (data.motion == M_FORWARD):
            self.color['data'][data.motion] = (241, 95, 95)

    def update_path(self, data):
        self.switch_marker[0] = True if data.right > 0 else False
        self.switch_marker[1] = True if data.left > 0 else False
        self.switch_marker[2] = True if data.forward > 0 else False

    def motorimagery(self, request):
        """Motorimagery를 위한 서비스를 관리한다"""
        # Cue를 발행한다.
        cue = MotorimageryCue()
        cue.header = copy.copy(request.header)
        self.publisher_motorimagery_cue.publish(cue)

        # 답변을 확인한다.
        while self.motorimagery_header.stamp < cue.header.stamp:
            rospy.sleep(self.spin_cycle)
        return {'header': self.motorimagery_header,
                'dir': self.motorimagery_result.dir}

    def update_motorimagery_result(self, data):
        """획득한 motorimagery를 기록한다"""
        self.motorimagery_header.stamp = rospy.Time.now()
        self.motorimagery_result = copy.copy(data)

    def __del__(self):
        """종료한다"""
        pygame.quit()


class Keyboard:
    """키보드 입력을 관리한다"""

    def __init__(self):
        """초기화"""
        # 파라미터 설정
        self.spin_cycle = rospy.Duration(rospy.get_param('~spin_cycle', 0.1))

        # 입출력 설정
        self.publisher_motorimagery_result = rospy.Publisher('interf/motorimagery_result', MotorimageryResult, queue_size=1)
        self.publisher_eyeblink_result = rospy.Publisher('interf/eyeblink_result', EyeblinkResult, queue_size=1)

        # 초기화
        self.key_setting = termios.tcgetattr(sys.stdin)
        self.key_watcher = rospy.Timer(self.spin_cycle, self.spin)

    def get_key(self):
        """키보드 입력을 획득한다"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.key_setting)
        return key

    def spin(self, event):
        """키보드 입력을 메시지로 발행한다"""
        key = self.get_key()
        if key == '\x03':   # ctrl+c
            self.key_watcher.shutdown()
        elif key == 'a':
            self.publisher_motorimagery_result.publish(M_LEFT)
        elif key == 'w':
            self.publisher_motorimagery_result.publish(M_FORWARD)
        elif key == 'd':
            self.publisher_motorimagery_result.publish(M_RIGHT)
        elif key == 's':
            self.publisher_motorimagery_result.publish(M_STOP)
        elif key == 'x':
            self.publisher_motorimagery_result.publish(M_BACKWARD)
        elif key == '2':
            self.publisher_eyeblink_result.publish(2)
        elif key == '3':
            self.publisher_eyeblink_result.publish(3)
        else:
            return


if __name__ == '__main__':
    rospy.init_node('interfacer')
    i = Interface()
    k = Keyboard()
    rospy.spin()
