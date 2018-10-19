#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import copy

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class INTERFACE_VISUALIZER:
    """인터페이스를 시각화한다."""
    def __init__(self):
        self.gvg_node = Marker()
        self.gvg_edge = Marker()

        rospy.Subscriber('gvg', MarkerArray, self.load_spatial_info)

        self.publisher = rospy.Publisher('interface', MarkerArray, queue_size=1)

        rospy.Timer(rospy.Duration(rospy.get_param('~publish_cycle', 1.0)), self.publish)

    def load_spatial_info(self, data):
        self.gvg_node = copy.deepcopy(data.markers[0])
        self.gvg_edge = copy.deepcopy(data.markers[1])

    def publish(self, event):
        interf = []
        try:                                # GVG 마커를 추가한다.
            interf.append(self.gvg_node)
            interf.append(self.gvg_edge)
        except: pass

        self.publisher.publish(interf)      # 마커를 발행한다.


if __name__ == '__main__':
    rospy.init_node('interface_visualizer')
    interface_visualizer = INTERFACE_VISUALIZER()
    rospy.spin()
