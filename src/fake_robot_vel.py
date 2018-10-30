#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import math
import numpy
import tf

from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Int32


class FAKE_ROBOT_VEL:
    """로봇의 자세와 상태를 임의로 생성한다"""
    def __init__(self):
        self.init_param()
        self.state = 0                                  # 로봇의 상태: 0=정지, 1=이동중
        self.cmd_vel = Twist()

        rospy.Subscriber('cmd_vel', Twist, self.update_cmd)

        self.publisher_state = rospy.Publisher('robot/state', Int32, queue_size=1)
        self.publisher_pose = rospy.Publisher('robot/pose', Pose, queue_size=1)

        self.broadcaster = tf.TransformBroadcaster()

        rospy.Timer(rospy.Duration(self.sim_cycle), self.navigation)

    def init_param(self):
        """파라미터를 초기화한다"""
        self.pose = Pose()                              # 로봇의 자세
        self.pose.position.x = rospy.get_param('~pose_x', 0.0)
        self.pose.position.y = rospy.get_param('~pose_y', 0.0)
        q = tf.transformations.quaternion_from_euler(0, 0, rospy.get_param('~pose_th', 0.0)*math.pi/180)
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]

        self.velocity_lin = rospy.get_param('~velocity_lin', 0.26)
        self.velocity_ang = rospy.get_param('~velocity_ang', 1.82)
        self.sim_cycle = rospy.get_param('~sim_cycle', 0.1)

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

    def update_cmd(self, data):
        """목표를 갱신한다"""
        self.cmd_vel = data

    def navigation(self, event):
        """목표를 향해 이동한다"""
        dd = self.cmd_vel.linear.x*self.sim_cycle       # 자세를 갱신한다.
        dth = self.cmd_vel.angular.z*self.sim_cycle

        th = tf.transformations.euler_from_quaternion([self.pose.orientation.x,
                                                       self.pose.orientation.y,
                                                       self.pose.orientation.z,
                                                       self.pose.orientation.w])[2]
        self.pose.position.x = dd*math.cos(th) + self.pose.position.x
        self.pose.position.y = dd*math.sin(th) + self.pose.position.y
        q = tf.transformations.quaternion_from_euler(0, 0, th + dth)
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]

        if dd == 0:                                     # 상태를 갱신한다.
            self.state = 0
        else:
            self.state = 1

        self.publisher_state.publish(self.state)        # 계산된 자세를 발행한다.
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
    rospy.init_node('fake_robot_vel')
    fake_robot_vel = FAKE_ROBOT_VEL()
    rospy.spin()
