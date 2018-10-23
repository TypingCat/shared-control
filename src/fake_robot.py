#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import math
import tf

from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray, Marker


class FAKE_ROBOT:
    """로봇의 자세와 상태를 임의로 생성한다"""
    def __init__(self):
        self.init_param()
        self.state = 0          # 로봇의 상태: 0=정지, 1=이동중
        self.lowspeed = 0.1

        rospy.Subscriber('robot/target', Pose, self.update_target)

        self.publisher_state = rospy.Publisher('robot/state', Int32, queue_size=1)
        self.publisher_pose = rospy.Publisher('robot/pose', Pose, queue_size=1)
        self.publisher_marker = rospy.Publisher('robot/marker', MarkerArray, queue_size=1)

        rospy.Timer(rospy.Duration(self.sim_cycle), self.navigation)

    def init_param(self):
        """파라미터를 초기화한다"""
        self.pose = Pose()      # 로봇의 자세
        self.pose.position.x = rospy.get_param('~pos_x', 0.0)
        self.pose.position.y = rospy.get_param('~pos_y', 0.0)
        q = tf.transformations.quaternion_from_euler(0, 0, rospy.get_param('~pos_th', 0.0)*math.pi/180)
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]

        self.target = Pose()    # 로봇의 목표 자세
        self.target.position.x = rospy.get_param('~pos_x', 0.0)
        self.target.position.y = rospy.get_param('~pos_y', 0.0)
        q = tf.transformations.quaternion_from_euler(0, 0, rospy.get_param('~pos_th', 0.0)*math.pi/180)
        self.target.orientation.x = q[0]
        self.target.orientation.y = q[1]
        self.target.orientation.z = q[2]
        self.target.orientation.w = q[3]

        self.velocity_lin = rospy.get_param('~velocity_lin', 0.26)
        self.velocity_ang = rospy.get_param('~velocity_ang', 1.82)
        self.margin_lin = rospy.get_param('~margin_lin', 0.1)
        self.margin_ang = rospy.get_param('~margin_ang', 0.1)

        self.sim_cycle = rospy.get_param('~sim_cycle', 0.1)

    def update_target(self, data):
        """목표 자세를 갱신한다"""
        self.target = data

    def navigation(self, event):
        """목표를 향해 이동한다"""
        dx = self.target.position.x - self.pose.position.x  # 상대자세를 확인한다.
        dy = self.target.position.y - self.pose.position.y
        target_th = tf.transformations.euler_from_quaternion([self.target.orientation.x,
                                                              self.target.orientation.y,
                                                              self.target.orientation.z,
                                                              self.target.orientation.w])[2]
        robot_th = tf.transformations.euler_from_quaternion([self.pose.orientation.x,
                                                             self.pose.orientation.y,
                                                             self.pose.orientation.z,
                                                             self.pose.orientation.w])[2]
        dth = (target_th - robot_th + math.pi)%(2*math.pi) - math.pi

        dd = math.sqrt(dx**2 + dy**2)
        if abs(dth) > self.margin_ang:
            self.state = 1
            if dth > 0:
                robot_th = robot_th + self.velocity_ang*self.sim_cycle
            else:
                robot_th = robot_th - self.velocity_ang*self.sim_cycle
            q = tf.transformations.quaternion_from_euler(0, 0, robot_th)
            self.pose.orientation.x = q[0]
            self.pose.orientation.y = q[1]
            self.pose.orientation.z = q[2]
            self.pose.orientation.w = q[3]

        elif dd > self.margin_lin:
            self.state = 1
            step_lin = self.velocity_lin*self.sim_cycle
            self.pose.position.x = self.pose.position.x + step_lin*math.cos(robot_th)
            self.pose.position.y = self.pose.position.y + step_lin*math.sin(robot_th)

            if dth > 0:
                robot_th = robot_th + self.lowspeed*self.velocity_ang*self.sim_cycle
            else:
                robot_th = robot_th - self.lowspeed*self.velocity_ang*self.sim_cycle
            q = tf.transformations.quaternion_from_euler(0, 0, robot_th)
            self.pose.orientation.x = q[0]
            self.pose.orientation.y = q[1]
            self.pose.orientation.z = q[2]
            self.pose.orientation.w = q[3]

        else:
            self.state = 0

        self.publisher_state.publish(self.state)
        self.publisher_pose.publish(self.pose)

        self.visualize_robot()

    def visualize_robot(self):
        """로봇의 자세를 화살표로 시각화한다"""
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
            self.publisher_marker.publish([robot, target])
        except: pass


if __name__ == '__main__':
    rospy.init_node('fake_robot')
    fake_robot = FAKE_ROBOT()
    rospy.spin()
