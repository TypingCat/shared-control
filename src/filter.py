#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import copy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class Filter:
    def __init__(self):
        
        # 파라미터 설정
        self.imu_in = rospy.get_param('~imu_in', 'imu_raw')
        self.imu_out = rospy.get_param('~imu_out', 'imu')
        self.imu_delayed = rospy.get_param('~imu_delayed', 'false')

        self.odom_in = rospy.get_param('~odom_in', 'odom_raw')
        self.odom_out = rospy.get_param('~odom_out', 'odom')
        self.odom_no_cov = rospy.get_param('~odom_no_cov', 'false')

        # IMU
        self.imu_time = rospy.Time.now()
        self.publisher_imu = rospy.Publisher(self.imu_out, Imu, queue_size=1)
        self.publisher_odom = rospy.Publisher(self.odom_out, Odometry, queue_size=1)
        if self.imu_delayed:
            rospy.Subscriber(self.imu_in, Imu, self.imu_filter)
        if self.odom_no_cov:
            rospy.Subscriber(self.odom_in, Odometry, self.odom_filter)
        

    def imu_filter(self, data):
        """뒤늦게 발행된 토픽은 무시한다."""
        if self.imu_time < data.header.stamp:
            self.imu_time = data.header.stamp
            self.publisher_imu.publish(data)
    
    def odom_filter(self, data):
        cov = 0.001
        data.twist.covariance = (cov, 0, 0, 0, 0, 0,
                                 0, cov, 0, 0, 0, 0,
                                 0, 0, cov, 0, 0, 0,
                                 0, 0, 0, cov, 0, 0,
                                 0, 0, 0, 0, cov, 0,
                                 0, 0, 0, 0, 0, cov)
        data.pose.covariance = (cov, 0, 0, 0, 0, 0,
                                0, cov, 0, 0, 0, 0,
                                0, 0, cov, 0, 0, 0,
                                0, 0, 0, cov, 0, 0,
                                0, 0, 0, 0, cov, 0,
                                0, 0, 0, 0, 0, cov)
        self.publisher_odom.publish(data)


if __name__ == '__main__':
    rospy.init_node('filter')
    f = Filter()
    rospy.spin()
