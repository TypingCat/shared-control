#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import math
import numpy
import tf

from geometry_msgs.msg import Pose, Point, TransformStamped
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray, Marker


class FAKE_ROBOT_POS:
    """로봇의 자세와 상태를 임의로 생성한다"""
    def __init__(self):
        self.init_param()
        self.state = 0                  # 로봇의 상태: 0=정지, 1=이동중

        rospy.Subscriber('robot/target', Pose, self.update_target)

        self.publisher_state = rospy.Publisher('robot/state', Int32, queue_size=1)
        self.publisher_pose = rospy.Publisher('robot/pose', Pose, queue_size=1)

        self.broadcaster = tf.TransformBroadcaster()

        rospy.Timer(rospy.Duration(self.sim_cycle), self.navigation)

    def init_param(self):
        """파라미터를 초기화한다"""
        self.pose = Pose()      # 로봇의 자세
        self.pose.position.x = rospy.get_param('~pose_x', 0.0)
        self.pose.position.y = rospy.get_param('~pose_y', 0.0)
        q = tf.transformations.quaternion_from_euler(0, 0, rospy.get_param('~pose_th', 0.0)*math.pi/180)
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]

        self.target = Pose()    # 로봇의 목표 자세
        self.target.position.x = rospy.get_param('~pose_x', 0.0)
        self.target.position.y = rospy.get_param('~pose_y', 0.0)
        q = tf.transformations.quaternion_from_euler(0, 0, rospy.get_param('~pose_th', 0.0)*math.pi/180)
        self.target.orientation.x = q[0]
        self.target.orientation.y = q[1]
        self.target.orientation.z = q[2]
        self.target.orientation.w = q[3]

        self.velocity_lin = rospy.get_param('~velocity_lin', 0.26)
        self.velocity_ang = rospy.get_param('~velocity_ang', 1.82)
        self.margin_lin = rospy.get_param('~margin_lin', 0.1)
        self.margin_ang = rospy.get_param('~margin_ang', 0.1)
        self.sim_cycle = rospy.get_param('~sim_cycle', 0.1)
        self.oscillation = rospy.get_param('~oscillation', 0.01)

        self.init_pos = (self.pose.position.x,
                         self.pose.position.y,
                         self.pose.position.z)
        self.init_ang = (self.pose.orientation.x,
                         self.pose.orientation.y,
                         self.pose.orientation.z,
                         self.pose.orientation.w)
        init_tf = numpy.dot(tf.transformations.translation_matrix(self.init_pos),
                            tf.transformations.quaternion_matrix(self.init_ang))
        self.init_tf_inv = tf.transformations.inverse_matrix(init_tf)

    def update_target(self, data):
        """목표를 갱신한다"""
        self.target = data
        robot_th = tf.transformations.euler_from_quaternion([data.orientation.x,
                                                             data.orientation.y,
                                                             data.orientation.z,
                                                             data.orientation.w])[2]

    def navigation(self, event):
        """목표를 향해 이동한다"""
        dx = self.target.position.x - self.pose.position.x  # 상대자세를 확인한다.
        dy = self.target.position.y - self.pose.position.y
        dd = math.sqrt(dx**2 + dy**2)

        target_th = tf.transformations.euler_from_quaternion([self.target.orientation.x,
                                                              self.target.orientation.y,
                                                              self.target.orientation.z,
                                                              self.target.orientation.w])[2]
        robot_th = tf.transformations.euler_from_quaternion([self.pose.orientation.x,
                                                             self.pose.orientation.y,
                                                             self.pose.orientation.z,
                                                             self.pose.orientation.w])[2]
        dth = (target_th - robot_th + math.pi)%(2*math.pi) - math.pi

        if abs(dth) > self.margin_ang:                      # 방향각을 조정한다.
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

        elif dd > self.margin_lin:                          # 위치를 조정한다.
            self.state = 1
            step_lin = self.velocity_lin*self.sim_cycle
            self.pose.position.x = self.pose.position.x + step_lin*math.cos(robot_th)
            self.pose.position.y = self.pose.position.y + step_lin*math.sin(robot_th)

            if dth > 0:
                robot_th = robot_th + self.oscillation*self.velocity_ang*self.sim_cycle
            else:
                robot_th = robot_th - self.oscillation*self.velocity_ang*self.sim_cycle
            q = tf.transformations.quaternion_from_euler(0, 0, robot_th)
            self.pose.orientation.x = q[0]
            self.pose.orientation.y = q[1]
            self.pose.orientation.z = q[2]
            self.pose.orientation.w = q[3]

        else:                                               # 목표에 도달했다면 대기한다.
            self.state = 0

        self.publisher_state.publish(self.state)            # 계산된 자세를 발행한다.
        self.publisher_pose.publish(self.pose)
        self.broadcast_tf(self.pose)

    def broadcast_tf(self, pose):
        """좌표계를 발행한다"""
        pos = (pose.position.x,
               pose.position.y,
               pose.position.z)
        ang = (pose.orientation.x,
               pose.orientation.y,
               pose.orientation.z,
               pose.orientation.w)
        tf_new = numpy.dot(tf.transformations.translation_matrix(pos),
                            tf.transformations.quaternion_matrix(ang))

        tf_diff = numpy.dot(self.init_tf_inv, tf_new)   # 좌표계: map-odom-base_footprint
        _, _, a, t, _ = tf.transformations.decompose_matrix(tf_diff)

        self.broadcaster.sendTransform(self.init_pos, self.init_ang, rospy.Time.now(), "odom", "map")
        self.broadcaster.sendTransform(t, tf.transformations.quaternion_from_euler(a[0], a[1], a[2]), rospy.Time.now(), "base_footprint", "odom")


if __name__ == '__main__':
    rospy.init_node('fake_robot_pos')
    fake_robot_pos = FAKE_ROBOT_POS()
    rospy.spin()
