#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import math

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped
from std_msgs.msg import Int32, ColorRGBA

from shared_control.msg import MID


class Visualize:
    """인터페이스를 시각화한다."""
    def __init__(self):
        self.gvg_node = Marker()
        self.gvg_edge = Marker()
        self.MI_marker_len = rospy.get_param('~MI_marker_len', 1.0)
        self.MI_points_L = []
        self.MI_points_R = []
        self.MI_marker_state = -1          # -1=비활성, 0=활성, else=점멸
        self.dst = Point()
        self.pose = Pose()

        rospy.Subscriber('visual/douser', Int32, self.update_douser)
        rospy.Subscriber('visual/MID_L', MID, self.update_flicker_L)
        rospy.Subscriber('visual/MID_R', MID, self.update_flicker_R)
        rospy.Subscriber('visual/MID_confirm', MID, self.update_lighter)
        rospy.Subscriber('visual/destination', Point, self.update_dst)
        rospy.Subscriber('robot/pose', PoseWithCovarianceStamped, self.update_pose)

        self.publisher = rospy.Publisher('visual', MarkerArray, queue_size=1)

        rospy.Timer(rospy.Duration(rospy.get_param('~publish_cycle', 0.3)), self.publish)

    def update_flicker_L(self, data):
        """왼쪽 마커를 점멸시킨다"""
        p1 = Point()
        p1.x = 0.3*math.cos(data.th) + data.point.x
        p1.y = 0.3*math.sin(data.th) + data.point.y
        p1.z = 0.1
        p2 = Point()
        p2.x = self.MI_marker_len*math.cos(data.th) + data.point.x
        p2.y = self.MI_marker_len*math.sin(data.th) + data.point.y
        p2.z = 0.1

        self.MI_points_L = [p1, p2]
        self.MI_marker_state = 1

    def update_flicker_R(self, data):
        """오른쪽 마커를 점멸시킨다"""
        p1 = Point()
        p1.x = 0.3*math.cos(data.th) + data.point.x
        p1.y = 0.3*math.sin(data.th) + data.point.y
        p1.z = 0.1
        p2 = Point()
        p2.x = self.MI_marker_len*math.cos(data.th) + data.point.x
        p2.y = self.MI_marker_len*math.sin(data.th) + data.point.y
        p2.z = 0.1

        self.MI_points_R = [p1, p2]
        self.MI_marker_state = 1

    def update_lighter(self, data):
        """왼쪽 마커를 점멸시킨다"""
        p1 = Point()
        p1.x = 0.3*math.cos(data.th) + data.point.x
        p1.y = 0.3*math.sin(data.th) + data.point.y
        p1.z = 0.1
        p2 = Point()
        p2.x = self.MI_marker_len*math.cos(data.th) + data.point.x
        p2.y = self.MI_marker_len*math.sin(data.th) + data.point.y
        p2.z = 0.1

        self.MI_points_L = [p1, p2]
        self.MI_marker_state = 0

    def update_douser(self, data):
        """마커를 비활성화한다"""
        self.MI_marker_state = -1

    def update_dst(self, data):
        """목적지를 갱신한다"""
        self.dst.x = data.x
        self.dst.y = data.y
        self.dst.z = data.z                 # 높이가 아닌 margin으로 사용한다.

    def update_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.pose = data.pose.pose

    def publish(self, event):
        """마커를 발행한다"""
        interf = []

        circle = Marker()                   # 목적지 마커를 추가한다.
        circle.header.stamp = rospy.Time.now()
        circle.header.frame_id = 'map'
        circle.id = 2
        circle.type = Marker.CYLINDER
        circle.action = Marker.ADD
        circle.scale.x = 2*self.dst.z
        circle.scale.y = 2*self.dst.z
        circle.scale.z = 0.01
        circle.pose.position.x = self.dst.x
        circle.pose.position.y = self.dst.y
        circle.pose.position.z = 0.02
        circle.color.g = 1.0
        circle.color.a = 0.5
        interf.append(circle)

        compass = Marker()                  # 목적지를 가리키는 마커를 추가한다.
        compass.header.stamp = rospy.Time.now()
        compass.header.frame_id = 'map'
        compass.id = 3
        compass.type = Marker.LINE_STRIP
        compass.action = Marker.ADD
        compass.scale.x = 0.02
        p1 = Point()
        p1.x = self.pose.position.x
        p1.y = self.pose.position.y
        p1.z = 0.01
        compass.points.append(p1)
        p2 = Point()
        p2.x = self.dst.x
        p2.y = self.dst.y
        p2.z = 0.01
        compass.points.append(p2)
        compass.color.g = 1.0
        compass.color.a = 0.5
        interf.append(compass)

        arrow1 = Marker()                   # MI 선택지 마커를 추가한다.
        arrow1.header.stamp = rospy.Time.now()
        arrow1.header.frame_id = 'map'
        arrow1.id = 4
        arrow1.type = Marker.ARROW
        arrow1.action = Marker.ADD
        arrow1.scale.x = 0.05
        arrow1.scale.y = 0.1
        arrow2 = Marker()
        arrow2.header.stamp = rospy.Time.now()
        arrow2.header.frame_id = 'map'
        arrow2.id = 5
        arrow2.type = Marker.ARROW
        arrow2.action = Marker.ADD
        arrow2.scale.x = 0.05
        arrow2.scale.y = 0.1

        if self.MI_marker_state > 0:        # 상태에 따라 MI 선택지를 변형한다.
            self.MI_marker_state = self.MI_marker_state + 1
            arrow1.points = self.MI_points_L
            arrow1.color.r = 1.0
            if self.MI_marker_state%2 != 1:
                arrow1.color.a = 0.5
            else:
                arrow1.color.a = 0.2
            arrow2.points = self.MI_points_R
            arrow2.color.b = 1.0
            if self.MI_marker_state%2 == 1:
                arrow2.color.a = 0.5
            else:
                arrow2.color.a = 0.2

        elif self.MI_marker_state == 0:
            arrow1.points = self.MI_points_L
            arrow1.color.g = 1.0
            arrow1.color.a = 0.5
            arrow2.points = self.MI_points_R
            arrow2.color.b = 1.0
            arrow2.color.a = 0.0

        else:
            p11 = Point()
            p12 = Point()
            arrow1.points = [p11, p12]
            arrow1.color.g = 1.0
            arrow1.color.a = 0.0
            p21 = Point()
            p22 = Point()
            arrow2.points = [p21, p22]
            arrow2.color.b = 1.0
            arrow2.color.a = 0.0

        interf.append(arrow1)
        interf.append(arrow2)
        self.publisher.publish(interf)      # 마커를 발행한다.


if __name__ == '__main__':
    rospy.init_node('visualizer')
    v = Visualize()
    rospy.spin()
