#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import numpy
import time

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker


class GVD:
    """Generalized Voronoi Diagram"""
    def __init__(self):
        self.map = OccupancyGrid()
        self.gvd = OccupancyGrid()

        rospy.Subscriber('map', OccupancyGrid, self.load_map)
        self.publisher = rospy.Publisher('gvd', OccupancyGrid, queue_size=1)

        rospy.Timer(rospy.Duration(2), self.publish)

    def load_map(self, data):
        """지도를 불러온다"""
        self.map.header = data.header                               # 지도를 획득한다.
        self.map.info = data.info
        self.map.data = numpy.array(data.data)

        t = time.time()                                             # GVD를 계산한다.
        self.gvd = self.calc_gvd(self.map.data.reshape(data.info.height,data.info.width),
                                 rospy.get_param('~PM', 10.0),
                                 rospy.get_param('~BM', 0.187/self.map.info.resolution))
        # rospy.loginfo('지도 크기 %dx%d'%(data.info.height, data.info.width))
        # rospy.loginfo('GVD 연산시간 %.2f초'%(time.time() - t))

    def calc_gvd(self, data, PM, BM):
        """Brushfire-based AGVD calculation"""
        frontier = []                                   # 연산을 초기화한다.
        origin = numpy.ones([len(data), len(data[0]), 2])*(-1)
        brushfire = numpy.ones([len(data), len(data[0])])*(-1)
        gvd = numpy.zeros([len(data), len(data[0])])
        for i in range(0, len(data)):
            for j in range(0, len(data[0])):
                # if data[i][j] > 50:
                if (data[i][j] > 50)|(data[i][j] < 0):
                    frontier.append([i, j])
                    origin[i][j][0] = i
                    origin[i][j][1] = j
                    brushfire[i][j] = 0
        neighbor = [          [0,  1],
                    [-1,  0],          [1,  0],
                              [0, -1]         ]
        step = 0

        while len(frontier) != 0:                       # 연산경계가 사라질 때까지
            step = step + 1
            next_frontier = []
            for x in frontier:
                for n in neighbor:
                    i = n[0] + x[0]                     # 장애물이 아닌 이웃의 위치를 확인한다.
                    j = n[1] + x[1]
                    try:
                        # if data[i][j] > 50:
                        if (data[i][j] > 50)|(data[i][j] < 0):
                            continue
                    except: continue

                    if brushfire[i][j] == -1:           # 연산한 적이 없는 이웃일 경우,
                        brushfire[i][j] = step          # 현재 스텝을 그대로 등록한다.
                        origin[i][j][0] = origin[x[0]][x[1]][0]
                        origin[i][j][1] = origin[x[0]][x[1]][1]
                        next_frontier.append([i, j])

                    elif brushfire[i][j] == step:       # 이번회차에서 연산된 이웃일 경우,
                        dist = abs(origin[i][j][0]-origin[x[0]][x[1]][0]) +\
                               abs(origin[i][j][1]-origin[x[0]][x[1]][1])
                        if (dist > PM)&(brushfire[x[0]][x[1]] > BM):
                            gvd[i][j] = 100             # 이웃을 GVD에 등록한다.

                    elif brushfire[i][j] == step - 1:   # 저번회차에서 연산된 이웃일 경우,
                        dist = abs(origin[i][j][0]-origin[x[0]][x[1]][0]) +\
                               abs(origin[i][j][1]-origin[x[0]][x[1]][1])
                        if (dist > PM)&(brushfire[x[0]][x[1]] > BM)&(gvd[i][j] != 100):
                            gvd[x[0]][x[1]] = 100       # 현재위치를 GVD에 등록한다.

            frontier = next_frontier                    # 경계를 갱신한다.

        # for i in range(0, len(data)):                   # Unknown 지역의 GVD를 제거한다.
        #     for j in range(0, len(data[0])):
        #         if (data[i][j] == -1)&(gvd[i][j] == 100):
        #             gvd[i][j] = 0

        return gvd

    def publish(self, event):
        """GVD를 점유격자로 출력한다"""
        try:
            gvd = OccupancyGrid()
            gvd.header.stamp = rospy.Time.now()
            gvd.header.frame_id = 'map'
            gvd.info = self.map.info
            gvd.data = self.gvd.flatten()
            self.publisher.publish(gvd)
        except: pass


if __name__ == '__main__':
    rospy.init_node('gvd')
    gvd = GVD()
    rospy.spin()
