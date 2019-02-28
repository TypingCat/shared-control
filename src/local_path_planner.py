#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import copy
import math

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker


class LocalPathPlanner:
    """Vector Field Histogram(VFH)에 기반하여 이동로봇의 속도를 결정한다"""
    def __init__(self):
        self.vel_lin = rospy.get_param('~vel_lin', 0.26)
        self.vel_ang = rospy.get_param('~vel_ang', 1.82)
        self.robot_rad = rospy.get_param('~robot_rad', 0.22)

        self.VFH = {}
        self.VFH['bin_num'] = rospy.get_param('~num_VFH_bin', 64)
        self.VFH['threshold'] = rospy.get_param('~threshold_VFH', 0.3)
        self.VFH['bin_width'] = 2 * math.pi / self.VFH['bin_num']
        self.VFH['bin_range'] = [self.VFH['bin_width'] * x for x in range(self.VFH['bin_num'])]
        self.VFH['bin_range_left'] = [x + self.VFH['bin_width'] / 2. for x in self.VFH['bin_range']]
        self.VFH['bin_range_right'] = [x - self.VFH['bin_width'] / 2. for x in self.VFH['bin_range']]
        self.VFH['data'] = [1. for _ in range(self.VFH['bin_num'])]

        rospy.loginfo(self.VFH)

        self.target = Twist()
        rospy.Subscriber('command', Int32, self.update_target)
        rospy.Subscriber('scan', LaserScan, self.update_VFH)

        self.publisher_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.publisher_VFH = rospy.Publisher('VFH', MarkerArray, queue_size=1)

        rospy.Timer(rospy.Duration(rospy.get_param('~freq', 0.1)), self.spin)

    def update_target(self, data):
        """목표를 갱신한다"""
        if data.data == 0:      # left
            command = (self.vel_lin, self.vel_ang)
        elif data.data == 1:    # forward
            command = (self.vel_lin, 0.)
        elif data.data == 2:    # right
            command = (self.vel_lin, -self.vel_ang)
        else:                   # stop
            command = (0., 0.)


        # if data.data == 0:      # left
        #     self.target.linear.x = 0
        #     self.target.angular.z = self.vel_ang
        # elif data.data == 1:    # forward
        #     self.target.linear.x = self.vel_lin
        #     self.target.angular.z = 0
        # elif data.data == 2:    # right
        #     self.target.linear.x = 0
        #     self.target.angular.z = -self.vel_ang
        # elif data.data == 3:    # backward
        #     self.target.linear.x = -self.vel_lin
        #     self.target.angular.z = 0
        # elif data.data == 4:    # stop
        #     self.target.linear.x = 0
        #     self.target.angular.z = 0
        self.target.linear.x = command[0]
        self.target.angular.x = command[1]

    def update_VFH(self, data):
        """VFH를 갱신한다"""
        self.VFH['data'] = [data.range_max - self.robot_rad for _ in self.VFH['data']]
        ranges = [x - self.robot_rad for x in data.ranges]
        for i in range(len(ranges)):
            # 장애물과 가장 가까운 bin을 찾는다.
            rad = i * data.angle_increment
            diff = [abs(x - rad) for x in self.VFH['bin_range']]
            j = diff.index(min(diff))
            # 장애물과의 거리를 갱신한다.
            if self.VFH['data'][j] > ranges[i]:
                self.VFH['data'][j] = ranges[i]

        self.publish_VFH()

    def publish_VFH(self):
        """VFH를 마커로 발행한다"""
        markers = []
        for i in range(self.VFH['bin_num']):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'base_footprint'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.02
            p0 = Point()
            p0.z = 0.01
            marker.points.append(p0)
            p1 = Point()
            p1.x = self.VFH['data'][i] * math.cos(self.VFH['bin_range_right'][i])
            p1.y = self.VFH['data'][i] * math.sin(self.VFH['bin_range_right'][i])
            p1.z = 0.01
            marker.points.append(p1)
            p2 = Point()
            p2.x = self.VFH['data'][i] * math.cos(self.VFH['bin_range_left'][i])
            p2.y = self.VFH['data'][i] * math.sin(self.VFH['bin_range_left'][i])
            p2.z = 0.01
            marker.points.append(p2)
            marker.points.append(p0)
            if self.VFH['data'][i] > self.VFH['threshold']:
                marker.color.g = 1.0
            else:
                marker.color.r = 1.0
            marker.color.a = 1.0
            markers.append(marker)

        self.publisher_VFH.publish(markers)

    def spin(self, event):
        """목표를 실행한다"""
        twist = copy.copy(self.target)
        self.publisher_cmd_vel.publish(twist)


if __name__ == '__main__':
    rospy.init_node('local_path_planner')
    lpp = LocalPathPlanner()
    rospy.spin()
