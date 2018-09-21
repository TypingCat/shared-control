#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import math
import tf

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point


class FAKE_ROBOT:
    """로봇의 자세와 상태를 임의로 생성한다"""
    def __init__(self):
        self.state = 0          # 로봇의 상태: 0=정지, 1=이동중
        self.pose = Pose()      # 로봇의 자세
        self.target = Pose()    # 로봇의 목표 자세

        self.pose.position.x = rospy.get_param('~robot_x', 0.0)
        self.pose.position.y = rospy.get_param('~robot_y', 0.0)
        q = tf.transformations.quaternion_from_euler(0, 0, rospy.get_param('~robot_th', 0.0)*math.pi/180)
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]

        self.target.position.x = rospy.get_param('~robot_x', 0.0)
        self.target.position.y = rospy.get_param('~robot_y', 0.0)
        q = tf.transformations.quaternion_from_euler(0, 0, rospy.get_param('~robot_th', 0.0)*math.pi/180)
        self.target.orientation.x = q[0]
        self.target.orientation.y = q[1]
        self.target.orientation.z = q[2]
        self.target.orientation.w = q[3]

        rospy.Subscriber('robot/target', Pose, self.update_target)
        self.state_publisher = rospy.Publisher('robot/state', Int32, queue_size=1)
        self.pose_publisher = rospy.Publisher('robot/pose', Pose, queue_size=1)
        self.marker_publisher = rospy.Publisher('robot/marker', MarkerArray, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.fake_navigation)

    def update_target(self, data):
        """목표 자세를 갱신한다"""
        self.target = data

    def fake_navigation(self, event):
        """목표를 향해 이동한다"""
        dx = self.target.position.x - self.pose.position.x
        dy = self.target.position.y - self.pose.position.y
        if dx**2 + dy**2 > 0.01:    # 목표에 도달하지 못했다면 이동한다.
            self.state = 1

            if dx != 0:             # 다음 위치를 계산한다.
                th = math.atan2(dy, dx)
            elif dy > 0:
                th = math.pi/2
            else:
                th = math.pi*3/2
            self.pose.position.x = 0.01*math.cos(th) + self.pose.position.x
            self.pose.position.y = 0.01*math.sin(th) + self.pose.position.y
            q = tf.transformations.quaternion_from_euler(0, 0, th)
            self.pose.orientation.x = q[0]
            self.pose.orientation.y = q[1]
            self.pose.orientation.z = q[2]
            self.pose.orientation.w = q[3]

        else:                       # 목표에 도달하면 대기한다.
            self.state = 0
            self.pose.orientation = self.target.orientation

        rospy.loginfo(self.pose)

        try:
            self.state_publisher.publish(self.state)
            self.pose_publisher.publish(self.pose)
        except: pass

        self.visualize_robot()

    def visualize_robot(self):
        robot = Marker()        # 로봇 마커를 생성한다.
        robot.header.stamp = rospy.Time.now()
        robot.header.frame_id = 'map'
        robot.id = 0
        robot.type = Marker.ARROW
        robot.action = Marker.ADD
        robot.scale.x = 0.05
        robot.scale.y = 0.1
        p1 = Point()
        p1.x = self.pose.position.x
        p1.y = self.pose.position.y
        p1.z = 0.1
        p2 = Point()
        th = tf.transformations.euler_from_quaternion([self.pose.orientation.x,
                                                       self.pose.orientation.y,
                                                       self.pose.orientation.z,
                                                       self.pose.orientation.w])[2]
        p2.x = self.pose.position.x + 0.2*math.cos(th)
        p2.y = self.pose.position.y + 0.2*math.sin(th)
        p2.z = 0.1
        robot.points = [p1, p2]
        robot.color.r = 1.0
        robot.color.a = 0.7

        target = Marker()       # 타겟 마커를 생성한다.
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'map'
        target.id = 1
        target.type = Marker.ARROW
        target.action = Marker.ADD
        target.scale.x = 0.05
        target.scale.y = 0.1
        p1 = Point()
        p1.x = self.target.position.x
        p1.y = self.target.position.y
        p1.z = 0.1
        p2 = Point()
        th = tf.transformations.euler_from_quaternion([self.target.orientation.x,
                                                       self.target.orientation.y,
                                                       self.target.orientation.z,
                                                       self.target.orientation.w])[2]
        p2.x = self.target.position.x + 0.2*math.cos(th)
        p2.y = self.target.position.y + 0.2*math.sin(th)
        p2.z = 0.1
        target.points = [p1, p2]
        target.color.g = 1.0
        target.color.a = 0.7

        try:
            self.marker_publisher.publish([robot, target])
        except: pass


if __name__ == '__main__':
    rospy.init_node('fake_robot')
    fake_robot = FAKE_ROBOT()
    rospy.spin()
