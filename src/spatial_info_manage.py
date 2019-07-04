#!/usr/bin/env python
#-*-coding: utf-8-*-

import rospy
import numpy
import networkx

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Int32

from shared_control.srv import Nearest, Neighbors, Node
from reserved_words import *


class SpatialInfoManage:
    """지도로부터 GVG를 생성하고 관련 서비스를 제공한다."""

    def __init__(self):
        """초기화"""
        self.map = OccupancyGrid()
        self.gvd = OccupancyGrid()
        self.graph = networkx.Graph()

        print(C_YELLO + '\rSpatial info manager, GVG 서비스 준비중...' + C_END)
        rospy.Subscriber('map', OccupancyGrid, self.load_map)
        rospy.Service('gvg/nearest', Nearest, self.get_nearest)
        rospy.Service('gvg/neighbors', Neighbors, self.get_neighbors)
        rospy.Service('gvg/node', Node, self.get_node)
        print(C_YELLO + '\rSpatial info manager, GVG 서비스 시작' + C_END)

        self.publisher = rospy.Publisher('visual/graph', MarkerArray, queue_size=1)
        rospy.Timer(rospy.Duration(0.3), self.publish)
        print(C_GREEN + '\rSpatial info manager, 초기화 완료' + C_END)

    def load_map(self, data):
        """지도로 GVD, GVG를 계산한다"""

        self.map.header = data.header                               # 지도를 획득한다.
        self.map.info = data.info
        self.map.data = numpy.array(data.data)

        if len(rospy.get_param('~custom_edge_list_x1', [])) != 0:   # 엣지리스트로 그래프를 구축한다.
            self.graph = self.create_graph(rospy.get_param('~custom_edge_list_x1', []),
                                           rospy.get_param('~custom_edge_list_y1', []),
                                           rospy.get_param('~custom_edge_list_x2', []),
                                           rospy.get_param('~custom_edge_list_y2', []),
                                           data.info.width, data.info.height)

        else:                                                       # 지도의 GVD를 계산한다.
            gvd = self.calc_gvd(self.map.data.reshape(data.info.height, data.info.width),
                                rospy.get_param('~gvd_PM', 10.0),
                                rospy.get_param('~gvd_BM', 0.187/self.map.info.resolution))
            self.gvd.header = data.header
            self.gvd.info = data.info
            self.gvd.data = gvd.flatten()

            footprint = self.draw_footprint(gvd)
            gvg = self.extract_gvg(footprint)
            self.graph = self.pruning(gvg, rospy.get_param('~gvg_minimum_path_distance', 0.3)**2)

    def create_graph(self, X1, Y1, X2, Y2, width, height):
        """주어진 좌표로 그래프를 구성한다"""

        g = networkx.Graph()
        res = 100
        for i in range(0, len(X1)):     # 인덱스를 생성한다.
            idx1 = int(res*(Y1[i]+height/2)*width + res*(X1[i]+width/2))
            idx2 = int(res*(Y2[i]+height/2)*width + res*(X2[i]+width/2))

            g.add_edge(idx1, idx2)      # (x1, y1)--(x2, y2) 엣지를 구축한다.
            g.nodes[idx1]['pos'] = [X1[i], Y1[i]]
            g.nodes[idx2]['pos'] = [X2[i], Y2[i]]

        return g

    def calc_gvd(self, data, PM, BM):
        """Brushfire-based AGVD calculation"""

        frontier = []                                   # 연산을 초기화한다.
        origin = numpy.ones([len(data), len(data[0]), 2])*(-1)
        brushfire = numpy.ones([len(data), len(data[0])])*(-1)
        gvd = numpy.zeros([len(data), len(data[0])])
        for i in range(0, len(data)):
            for j in range(0, len(data[0])):
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

        return gvd

    def draw_footprint(self, gvd):
        """점유격자를 그래프로 변환한다"""

        footprint = networkx.Graph()                # 연산을 초기화한다.
        neighbor = [                   [1,  0],
                    [-1, -1], [0, -1], [1, -1]]

        for x in range(0, len(gvd)):                # 모든 노드를 연결한다.
            for y in range(0, len(gvd[0])):
                if gvd[x][y] == 100:
                    idx_xy = y*len(gvd) + x
                    for n in neighbor:
                        i = n[0] + x
                        j = n[1] + y
                        try:
                            if gvd[i][j] == 100:    # 노드의 위치를 계산한다.
                                px = (y+0.5)*self.gvd.info.resolution +\
                                     self.gvd.info.origin.position.x
                                py = (x+0.5)*self.gvd.info.resolution +\
                                     self.gvd.info.origin.position.y
                                pi = (j+0.5)*self.gvd.info.resolution +\
                                     self.gvd.info.origin.position.x
                                pj = (i+0.5)*self.gvd.info.resolution +\
                                     self.gvd.info.origin.position.y

                                idx_ij = j*len(gvd) + i
                                footprint.add_edge(idx_xy, idx_ij)
                                footprint.nodes[idx_xy]['pos'] = [px, py]
                                footprint.nodes[idx_ij]['pos'] = [pi, pj]
                                footprint.nodes[idx_xy]['state'] = -1
                                footprint.nodes[idx_ij]['state'] = -1
                                footprint.nodes[idx_xy]['origin'] = idx_xy
                                footprint.nodes[idx_ij]['origin'] = idx_ij
                        except: continue

        return footprint

    def extract_gvg(self, footprint):
        """GVD로부터 GVG를 추출한다"""

        seed = [-1, 0]                              # 가장 규모가 큰 서브그래프를 확인한다.
        foot = footprint.copy()
        while len(foot.nodes) > 0:
            for f in foot.nodes:
                if len(list(foot.neighbors(f))) == 1:
                    e = list(networkx.dfs_edges(foot, f))
                    toe = [f]
                    for _, n in e:
                        toe.append(n)
                    if seed[1] < len(toe):
                        seed = [f, len(toe)]
                    foot.remove_nodes_from(toe)     # 확인한 서브그래프는 foot에서 제거한다.
                    break
        seed = (seed[0],                            # 연산 시작점을 확보한다.
                list(footprint.neighbors(seed[0]))[0])

        footprint.nodes[seed[0]]['state'] = 1       # footprint 노드의 상태를 정의한다.
        footprint.nodes[seed[0]]['head'] = seed[0]
        for e in networkx.dfs_edges(footprint, seed[0]):
            neighbor = len(list(footprint.neighbors(e[1])))
            footprint.nodes[e[1]]['state'] = neighbor if (neighbor < 3) else 3
            footprint.nodes[e[1]]['head'] = -1

        gvg = networkx.Graph()                      # GVG를 초기화한다.
        gvg.add_edge(seed[0], seed[1])
        gvg.nodes[seed[0]]['pos'] = footprint.nodes[seed[0]]['pos']

        seed = set([seed])
        while len(seed) > 0:                                # 시드 리스트가 빌 때까지
            s = list(seed)[0]                               # 시드를 꺼낸다.

            if footprint.nodes[s[1]]['head'] == -1:         # 만약 첫 방문이라면,
                footprint.nodes[s[1]]['head'] = s[1]        # 노드 생성을 시작한다.
                pos_x = [footprint.nodes[s[1]]['pos'][0]]
                pos_y = [footprint.nodes[s[1]]['pos'][1]]
                hypha = set(list(footprint.neighbors(s[1]))) - set(s)
                visit = set([])
                edges = set([s])

                while len(hypha) > 0:                       # 시드의 모든 이웃을 방문한다.
                    h = list(hypha)[0]

                    if footprint.nodes[h]['state'] == footprint.nodes[s[1]]['state']:
                        if footprint.nodes[h]['head'] == -1:            # 이웃이 시드와 동류라면,
                            footprint.nodes[h]['head'] = s[1]           # 둘을 연결한다.
                            pos_x.append(footprint.nodes[h]['pos'][0])
                            pos_y.append(footprint.nodes[h]['pos'][1])
                            hypha.update(list(footprint.neighbors(h)))  # 이웃이웃을 방문예정에 포함시킨다.

                    else:                                               # 이웃이 시드와 상이하다면,
                        if footprint.nodes[h]['head'] != -1:            # 그리고 방문한 흔적이 발견된다면,
                            edges.add((s[1],                            # 시드와 연결한다.
                                       footprint.nodes[h]['head']))
                        else:
                            seed.add((s[1], h))                         # 아니라면 새로운 시드를 생성한다.

                    visit.add(h)                            # 중복 방문은 피한다.
                    hypha = hypha - visit

                gvg.add_edges_from(list(edges))             # 노드를 구축한다.
                gvg.nodes[s[1]]['pos'] = [sum(pos_x)/len(pos_x), sum(pos_y)/len(pos_y)]

            seed.remove(s)

        return gvg

    def pruning(self, gvg, minimum_path_distance):
        """GVG를 다듬는다"""

        knur = -1                                           # 다듬을 곳이 없을 때까지,
        while knur != 0:
            knur = 0
            node_list = []
            edge_list = []
            for n in gvg.nodes:                             # 그래프를 탐색한다.
                neighbor = list(gvg.neighbors(n))
                if len(neighbor) == 2:                      # 경로 노드는 불필요하다.
                    node_list.append(n)
                    edge_list.append(neighbor)
                    knur = 1
                    break
            gvg.add_edges_from(edge_list)
            gvg.remove_nodes_from(node_list)

        knur = -1                                           # 다듬을 곳이 없을 때까지,
        while knur != 0:
            knur = 0
            node_list = []
            edge_list = []
            for n in gvg.nodes:                             # 그래프를 탐색한다.
                neighbor = list(gvg.neighbors(n))
                if len(neighbor) == 1:                      # 짧은 말단은 불필요하다.
                    dist = (gvg.nodes[neighbor[0]]['pos'][0] - gvg.nodes[n]['pos'][0])**2 +\
                           (gvg.nodes[neighbor[0]]['pos'][1] - gvg.nodes[n]['pos'][1])**2
                    if dist < minimum_path_distance:
                        node_list.append(n)
                        knur = 1
                        break
            gvg.add_edges_from(edge_list)
            gvg.remove_nodes_from(node_list)

        knur = -1                                           # 다듬을 곳이 없을 때까지,
        while knur != 0:
            knur = 0
            node_list = []
            edge_list = []
            for n in gvg.nodes:                             # 그래프를 탐색한다.
                neighbor = list(gvg.neighbors(n))
                if len(neighbor) == 2:                      # 경로 노드는 불필요하다.
                    node_list.append(n)
                    edge_list.append(neighbor)
                    knur = 1
                    break
            gvg.add_edges_from(edge_list)
            gvg.remove_nodes_from(node_list)

        return gvg

    def publish(self, event):
        """GVG를 출력한다"""

        gvg_node = Marker()                             # GVG 노드 마커를 생성한다.
        gvg_node.header.stamp = rospy.Time.now()
        gvg_node.header.frame_id = 'map'
        gvg_node.id = 0
        gvg_node.type = Marker.POINTS
        gvg_node.action = Marker.ADD
        gvg_node.scale.x = 0.5*self.map.info.resolution
        gvg_node.scale.y = 0.5*self.map.info.resolution
        gvg_node.points = []
        gvg_node.colors = []
        for n in self.graph.nodes:
            c = ColorRGBA()
            c.a = 0.5
            c.r = 1
            gvg_node.colors.append(c)
            p = Point()
            p.x = self.graph.nodes[n]['pos'][0]
            p.y = self.graph.nodes[n]['pos'][1]
            p.z = 0.01
            gvg_node.points.append(p)

        gvg_edge = Marker()                             # GVG 엣지 마커를 생성한다.
        gvg_edge.header.stamp = rospy.Time.now()
        gvg_edge.header.frame_id = 'map'
        gvg_edge.id = 1
        gvg_edge.type = Marker.LINE_LIST
        gvg_edge.action = Marker.ADD
        gvg_edge.scale.x = 0.2*self.map.info.resolution
        gvg_edge.points = []
        gvg_edge.color.a = 0.7
        for e in self.graph.edges:
            p1 = Point()
            p1.x = self.graph.nodes[e[0]]['pos'][0]
            p1.y = self.graph.nodes[e[0]]['pos'][1]
            p1.z = 0.01
            gvg_edge.points.append(p1)
            p2 = Point()
            p2.x = self.graph.nodes[e[1]]['pos'][0]
            p2.y = self.graph.nodes[e[1]]['pos'][1]
            p2.z = 0.01
            gvg_edge.points.append(p2)

        self.publisher.publish([gvg_node, gvg_edge])     # GVG 마커를 출력한다.

    def get_nearest(self, request):
        """입력한 위치와 가장 가까운 GVG 노드의 id를 반환한다"""
        try:
            nearest = [-1, 2147483647]
            for n in self.graph.nodes:
                dist2 = (request.point.x - self.graph.nodes[n]['pos'][0])**2 +\
                        (request.point.y - self.graph.nodes[n]['pos'][1])**2
                if dist2 < nearest[1]:
                    nearest = [n, dist2]
            return {'id': nearest[0]}
        except:
            rospy.loginfo('서비스 get_nearest 실패')
            return {'id': -1}

    def get_neighbors(self, request):
        """입력한 id를 갖는 GVG 노드의 이웃노드 id 리스트를 반환한다"""
        try:
            return {'ids': list(self.graph.neighbors(request.id))}
        except:
            rospy.loginfo('서비스 get_neighbors 실패')
            return {'ids': [-1]}

    def get_node(self, request):
        """입력한 id를 갖는 노드의 속성을 반환한다"""
        p = Point()
        try:
            p.x = self.graph.nodes[request.id]['pos'][0]
            p.y = self.graph.nodes[request.id]['pos'][1]
            p.z = 0
        except:
            rospy.loginfo('서비스 get_node 실패')

        return {'point': p}


if __name__ == '__main__':
    rospy.init_node('spatial_info_manager')
    sim = SpatialInfoManage()
    rospy.spin()
