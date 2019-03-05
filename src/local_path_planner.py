#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import copy
import math

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker


class LocalPathPlanner:
    """Vector Field Histogram(VFH)(간략)에 기반하여 이동로봇의 속도를 결정한다"""
    def __init__(self):
        # self.robot_vel_lin = rospy.get_param('~robot_vel_lin', 0.26)
        # self.robot_vel_ang = rospy.get_param('~robot_vel_ang', 1.82)
        self.robot_rad = rospy.get_param('~robot_rad', 0.22)

        self.VFH = {}
        self.VFH['bin_num'] = rospy.get_param('~num_VFH_bin', 64)
        self.VFH['bin_width'] = 2 * math.pi / self.VFH['bin_num']
        self.VFH['bin_range'] = [self.VFH['bin_width'] * x for x in range(self.VFH['bin_num'])]
        self.VFH['bin_range_left'] = [x + self.VFH['bin_width'] / 2. for x in self.VFH['bin_range']]
        self.VFH['bin_range_right'] = [x - self.VFH['bin_width'] / 2. for x in self.VFH['bin_range']]
        self.VFH['data'] = [1. for _ in range(self.VFH['bin_num'])]
        self.VFH['is_free'] = [False for _ in range(self.VFH['bin_num'])]
        self.VFH['bound_num'] = int(round(self.VFH['bin_num'] * rospy.get_param('~VFH_bound', math.pi / 6) / (2 * math.pi)))
        self.VFH['threshold'] = rospy.get_param('~VFH_threshold', 0.3)

        self.cmd_vec = Twist()
        self.target = Twist()

        rospy.Subscriber('cmd_vec', Twist, self.update_cmd_vec)
        rospy.Subscriber('scan', LaserScan, self.update_VFH)

        self.publisher_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.publisher_VFH = rospy.Publisher('VFH', MarkerArray, queue_size=1)

        rospy.Timer(rospy.Duration(rospy.get_param('~freq', 0.1)), self.spin)

    def update_cmd_vec(self, data):
        """목표를 갱신한다"""
        self.cmd_vec = copy.copy(data)

    def update_VFH(self, data):
        """VFH를 갱신한다"""
        self.VFH['data'] = [data.range_max - self.robot_rad for _ in self.VFH['data']]
        ranges = [x - self.robot_rad for x in data.ranges]

        # 장애물과의 최소거리를 bin에 기록한다.
        for i in range(len(ranges)):
            rad = i * data.angle_increment
            diff = [abs(x - rad) for x in self.VFH['bin_range']]
            j = diff.index(min(diff))
            if self.VFH['data'][j] > ranges[i]:
                self.VFH['data'][j] = ranges[i]

        # 역치도달여부를 파악한다.
        for i in range(len(self.VFH['data'])):
            if self.VFH['data'][i] < self.VFH['threshold']:
                self.VFH['is_free'][i] = False
            else:
                self.VFH['is_free'][i] = True

        lin, ang = self.find_direction()
        self.target.linear.x = lin
        self.target.angular.z = ang / 10.

        print('선속도 {0}, 각속도 {1}'.format(self.target.linear.x, self.target.angular.z))

        self.visualize_VFH()

    def find_direction(self):
        """명령과 장애물을 고려하여 이동방향을 검색한다"""
        #
        #
        # 각도를 [-pi, pi]로 조정해야 한다.
        #
        #

        # 정지명령은 검색이 불필요하다.
        if self.cmd_vec.linear.x == 0:
            return 0, 0

        # 명령에 해당하는 bin을 검색한다.
        diff = [abs(x - self.cmd_vec.angular.z) % (2 * math.pi) for x in self.VFH['bin_range']]
        idx = diff.index(min(diff))
        left = idx
        right = idx

        # 해당 bin이 비어있을 경우 좌우 경계를 확인한다.
        if self.VFH['is_free'][idx]:
            for _ in range(2 * self.VFH['bound_num']):
                # print('{0}'.format(left))
                if self.VFH['is_free'][left + 1]:
                    left += 1
                else:
                    break
            # print('!!')
            for _ in range(2 * self.VFH['bound_num']):
                # print('{0}'.format(right))
                if self.VFH['is_free'][right - 1]:
                    right -= 1
                else:
                    break

            # print('왼쪽 {0}, 중앙 {1}, 오른쪽 {2}\n'.format(left, idx, right))
            center = (self.VFH['bin_range'][left] + self.VFH['bin_range'][right]) / 2.
            # return self.cmd_vec.linear.x, center
            return self.cmd_vec.linear.x, (center - math.pi) % (2 * math.pi) + math.pi

        # 중앙이 막혀있을경우,
        else:

            # 왼쪽의 공간을 확인한다.
            ll = idx
            lr = idx
            for i in range(self.VFH['bound_num']):
                if not self.VFH['is_free'][lr + 1]:
                    lr += 1
                else:
                    break
            if not lr == idx:
                ll = lr
                for i in range(2 * self.VFH['bound_num']):
                    if self.VFH['is_free'][ll + 1]:
                        ll += 1
                    else:
                        break
            lc = (self.VFH['bin_range'][ll] + self.VFH['bin_range'][lr]) / 2.

            # 오른쪽의 공간을 확인한다.
            rl = idx
            rr = idx
            for i in range(self.VFH['bound_num']):
                if not self.VFH['is_free'][rr - 1]:
                    rr -= 1
                else:
                    break
            if not rr == idx:
                rl = rr
                for i in range(2 * self.VFH['bound_num']):
                    if self.VFH['is_free'][rl - 1]:
                        rl -= 1
                    else:
                        break
            rc = (self.VFH['bin_range'][rl] + self.VFH['bin_range'][rr]) / 2.

            # 가까운 공간을 판단한다.
            # print('구현중 {0}, {1}, {2}'.format(lc, self.VFH['bin_range'][idx], rc))
            if lc == 0 and rc == 0:
                return 0, 0
            elif abs(lc - self.VFH['bin_range'][idx]) < abs(rc - self.VFH['bin_range'][idx]):
                # return self.cmd_vec.linear.x, lc
                return self.cmd_vec.linear.x, (lc - math.pi) % (2 * math.pi) + math.pi
            else:
                # return self.cmd_vec.linear.x, rc
                return self.cmd_vec.linear.x, (rc - math.pi) % (2 * math.pi) + math.pi

    def visualize_VFH(self):
        """VFH를 마커로 발행한다"""
        markers = []
        for i in range(self.VFH['bin_num']):
            # VFH bin을 삼각형으로 표현한다.
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

        # 사용자 명령을 화살표로 표현한다.
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'base_footprint'
        marker.id = self.VFH['bin_num']
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        p0 = Point()
        p0.z = 0.3
        marker.points.append(p0)
        p1 = Point()
        p1.x = self.cmd_vec.linear.x * math.cos(self.cmd_vec.angular.z)
        p1.y = self.cmd_vec.linear.x * math.sin(self.cmd_vec.angular.z)
        p1.z = 0.3
        marker.points.append(p1)
        marker.color.b = 1.0
        marker.color.a = 0.5
        markers.append(marker)

        # 이동목표를 화살표로 표현한다.
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'base_footprint'
        marker.id = self.VFH['bin_num'] + 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        p0 = Point()
        p0.z = 0.3
        marker.points.append(p0)
        p1 = Point()
        p1.x = self.target.linear.x * math.cos(self.target.angular.z)
        p1.y = self.target.linear.x * math.sin(self.target.angular.z)
        p1.z = 0.3
        marker.points.append(p1)
        marker.color.r = 1.0
        marker.color.a = 0.5
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
