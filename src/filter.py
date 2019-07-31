#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import copy

from sensor_msgs.msg import Imu


class Filter:
    def __init__(self):
        # 파라미터 설정
        self.imu_in = rospy.get_param('~imu_in', 'imu_raw')
        self.imu_out = rospy.get_param('~imu_out', 'imu')

        # IMU
        self.imu_time = rospy.Time.now()
        rospy.Subscriber(self.imu_in, Imu, self.check_imu)
        self.publisher_imu = rospy.Publisher(self.imu_out, Imu, queue_size=1)

    def check_imu(self, data):
        """뒤늦게 발행된 토픽은 무시한다."""
        if self.imu_time < data.header.stamp:
            self.imu_time = data.header.stamp
            self.publisher_imu.publish(data)


if __name__ == '__main__':
    rospy.init_node('filter')
    f = Filter()
    rospy.spin()
