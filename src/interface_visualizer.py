#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import copy

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int32, ColorRGBA


class INTERFACE_VISUALIZER:
    """인터페이스를 시각화한다."""
    def __init__(self):
        self.gvg_node = Marker()
        self.gvg_edge = Marker()
        self.marker_point = Point()
        self.marker_state = -1          # -1=비활성, 0=활성, else=점멸
        self.dst = Point()
        self.pose = Pose()

        rospy.Subscriber('interface/lighter', Point, self.update_lighter)
        rospy.Subscriber('interface/flicker', Point, self.update_flicker)
        rospy.Subscriber('interface/douser', Int32, self.update_douser)
        rospy.Subscriber('interface/destination', Point, self.update_dst)
        rospy.Subscriber('robot/pose', Pose, self.update_pose)

        self.publisher = rospy.Publisher('interface', MarkerArray, queue_size=1)

        rospy.Timer(rospy.Duration(rospy.get_param('~publish_cycle', 0.3)), self.publish)

    def update_lighter(self, data):
        """마커를 활성화한다"""
        self.marker_point.x = data.x
        self.marker_point.y = data.y
        self.marker_point.z = data.z
        self.marker_state = 0

    def update_flicker(self, data):
        """마커를 점멸시킨다"""
        self.marker_point.x = data.x
        self.marker_point.y = data.y
        self.marker_point.z = data.z
        self.marker_state = 1

    def update_douser(self, data):
        """마커를 비활성화한다"""
        self.marker_state = -1

    def update_dst(self, data):
        """목적지를 갱신한다"""
        self.dst.x = data.x
        self.dst.y = data.y
        self.dst.z = data.z                 # 높이가 아닌 margin으로 사용한다.

    def update_pose(self, data):
        """로봇의 자세를 갱신한다"""
        self.pose = data

    def publish(self, event):
        """마커를 발행한다"""
        interf = []
        if self.marker_state == 0:
            marker = Marker()               # 활성화 상태일 경우 원기둥마커를 추가한다.
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.id = 2
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 1.0
            marker.pose.position.x = self.marker_point.x
            marker.pose.position.y = self.marker_point.y
            marker.pose.position.z = 0.5
            marker.color.r = 1.0
            marker.color.a = 0.5
            interf.append(marker)

        elif self.marker_state > 0:         # 비활성화 상태일 경우 원기둥마커가 점멸하도록 설정한다.
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.id = 2
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 1.0
            marker.pose.position.x = self.marker_point.x
            marker.pose.position.y = self.marker_point.y
            marker.pose.position.z = 0.5
            marker.color.r = 1.0
            if self.marker_state%2 == 1:
                marker.color.a = 0.5
            else:
                marker.color.a = 0.2
            interf.append(marker)
            self.marker_state = self.marker_state + 1

        circle = Marker()                   # 목적지 마커를 추가한다.
        circle.header.stamp = rospy.Time.now()
        circle.header.frame_id = 'map'
        circle.id = 3
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
        compass.id = 4
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

        self.publisher.publish(interf)      # 마커를 발행한다.


if __name__ == '__main__':
    rospy.init_node('interface_visualizer')
    interface_visualizer = INTERFACE_VISUALIZER()
    rospy.spin()
