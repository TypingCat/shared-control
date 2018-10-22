#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import copy

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, ColorRGBA


class INTERFACE_VISUALIZER:
    """인터페이스를 시각화한다."""
    def __init__(self):
        self.gvg_node = Marker()
        self.gvg_edge = Marker()
        self.marker_point = Point()
        self.marker_state = 0           # -1=비활성, 0=활성, else=점멸

        rospy.Subscriber('gvg', MarkerArray, self.load_spatial_info)
        rospy.Subscriber('interface/lighter', Point, self.update_lighter)
        rospy.Subscriber('interface/flicker', Point, self.update_flicker)
        rospy.Subscriber('interface/douser', Int32, self.update_douser)

        self.publisher = rospy.Publisher('interface', MarkerArray, queue_size=1)

        rospy.Timer(rospy.Duration(rospy.get_param('~publish_cycle', 0.3)), self.publish)

    def load_spatial_info(self, data):
        """공간정보를 불러온다"""
        self.gvg_node = copy.deepcopy(data.markers[0])
        self.gvg_edge = copy.deepcopy(data.markers[1])

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

    def publish(self, event):
        """마커를 발행한다"""
        interf = []
        try:                                # GVG 마커를 추가한다.
            interf.append(self.gvg_node)
            interf.append(self.gvg_edge)
        except: pass

        if self.marker_state == 0:
            marker = Marker()               # 활성화 상태일 경우 원기둥마커를 추가한다.
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.id = 0
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = 0.3            # x축 직경
            marker.scale.y = 0.3            # y축 직경
            marker.scale.z = 1.0            # z축 높이
            marker.pose.position.x = self.marker_point.x
            marker.pose.position.y = self.marker_point.y
            marker.pose.position.z = 0.5
            marker.color.r = 1.0
            marker.color.a = 1.0

            interf.append(marker)

        elif self.marker_state > 0:
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.id = 0
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = 0.3            # x축 직경
            marker.scale.y = 0.3            # y축 직경
            marker.scale.z = 1.0            # z축 높이
            marker.pose.position.x = self.marker_point.x
            marker.pose.position.y = self.marker_point.y
            marker.pose.position.z = 0.5
            marker.color.r = 1.0
            if self.marker_state%2 == 1:    # 점멸 상태일 경우 원기둥마커를 주기적으로 추가한다.
                marker.color.a = 1.0
            else:
                marker.color.a = 0.5

            interf.append(marker)
            self.marker_state = self.marker_state + 1

        self.publisher.publish(interf)      # 마커를 발행한다.


if __name__ == '__main__':
    rospy.init_node('interface_visualizer')
    interface_visualizer = INTERFACE_VISUALIZER()
    rospy.spin()
