#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import pygame
import copy
import termios, sys, select, tty

from std_msgs.msg import Int32, Header
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Image

from shared_control.msg import MotorimageryCue, MotorimageryResult, EyeblinkResult
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
                               (255, 223, 36)],  # M_FORWARD}
                      'time': [rospy.get_time(),
                               rospy.get_time(),
                               rospy.get_time(),
                               rospy.get_time()]}

        # 입출력 설정
        print(C_YELLO + '\rInterfacer, BCI 서비스 준비중...' + C_END)
        self.publisher_motorimagery_cue = rospy.Publisher('interf/motorimagery_cue', MotorimageryCue, queue_size=1)
        self.motorimagery_header = Header()
        self.motorimagery_result = MotorimageryResult()
        rospy.Subscriber('interf/motorimagery_result', MotorimageryResult, self.update_motorimagery_result)
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
        self.draw_arrow(M_RIGHT, 70, 0.94*self.width, 0.5*self.height)
        self.draw_arrow(M_LEFT, 70, 0.06*self.width, 0.5*self.height)
        self.draw_arrow(M_FORWARD, 70, 0.5*self.width, 0.1*self.height)



        # 상태 표현
        # self.draw_circle()



        # 화면 출력
        pygame.display.flip()

    def draw_arrow(self, type, scale, x, y):
        """화살표를 그린다"""
        # 좌표 생성
        if type == M_RIGHT:
            arr = [[1, 0], [0, 1], [0, 0.5], [-1, 0.5], [-1, -0.5], [0, -0.5], [0, -1]]
        elif type == M_LEFT:
            arr = [[1, 0.5], [0, 0.5], [0, 1], [-1, 0], [0, -1], [0, -0.5], [1, -0.5]]
        elif type == M_FORWARD:
            arr = [[1, 0], [0.5, 0], [0.5, 1], [-0.5, 1], [-0.5, 0], [-1, 0], [0, -1]]
        arr = [[scale*i+x, scale*j+y] for [i, j] in arr]
        # 색상 결정
        if rospy.get_time() > self.color['time'][type]:
            self.color['data'][type] = self.color['data'][0]
        # 출력
        pygame.draw.polygon(self.screen, self.color['data'][type], arr)



    def draw_circle(self, type, scale, x, y, text):
        """원을 그린다"""
        pass



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
        # 마커 색상 변경
        self.color['data'][data.dir] = (241, 95, 95)
        self.color['time'][data.dir] = rospy.get_time() + 3.

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
