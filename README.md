# shared_control
본 패키지는 이동로봇의 공유제어를 담당한다. 이동로봇은 그래프를 참고하여 목적지를 결정하며, 그래프에서 교차로가 발생하면 BCI에게 선택을 요청하는 식으로 목적지에 도달한다.

패키지 관리자 노진홍
<br> 인턴, 지능로봇연구단, 한국과학기술연구원
<br> fini@kist.re.kr
<br> 02-958-4864


## 1. 개요
### 1.1. 목표
한국전자전 참가
- 일시: 2018.10.24(수)-27(토)
- 장소: COEX

### 1.2. 시나리오
1. 로봇에게는 지도가 주어진다. 로봇은 이 지도를 바탕으로 GVG를 구축한다.
2. 로봇은 자신의 위치를 GVG 수준에서 파악할 수 있으며, 현재 위치와 인접한 노드들을 이동대상으로 본다.
3. 로봇은 정지중일 경우,
    - Eye blink 신호를 받으면 움직일 방향을 motor imagery로 묻는다.
4. 로봇은 주행중일 경우,
    - Eye blink 신호를 받으면 정지한다.
    - 교차로에 진입하면 정지하고, 움직일 방향을 motor imagery로 묻는다.
    - 말단에 진입하면 정지한다.

### 1.3. 협업 구조
#### 김래현박사님팀
- 실무자: 김다혜, dahyekim@kist.re.kr
- 역할: BCI로 사용자의 의도를 획득한다.

| 노드 | 수신 | 발신 |
|-|-|-|
| Motor imagery | Binary question | Binary answer |
| Eye blink | | trigger |

#### 최종석박사님팀
- 실무자: 노진홍, fini@kist.re.kr
- 역할: BCI-이동로봇을 위한 공유제어를 설계한다.

| 노드 | 수신 | 발신 |
|-|-|-|
| Task planner | binary answer, gvg answer, robot state, robot pose  | binary question, gvg question, choice info., target pose |
| Spatial info. manager | gvg question, map | gvg answer, gvg |
| Map server | map | map |
| Plan visualizer | choice info. | marker |
| Rviz | map, gvg, robot pose, marker | |

#### 윤상석교수님팀
- 실무자: 엄홍규, ehg2y@naver.com
- 역할: 이동로봇을 제어한다.

| 노드 | 수신 | 발신 |
|-|-|-|
| Turtlebot3 core | velocity | coordinate |
| Turtlebot3 lds | | obstacle |
| Camera | | video |
| SLAM | coordinate, obstacle | map |
| Localization | coordinate, obstacle, map | robot pose |
| Navigation | robot pose, map | velocity |
| Motion manager | target pose | robot state, command |


## 2. 개발환경
### 2.1. 소프트웨어
- Ubuntu 16.04
- ROS kinetic
- Python 2.7
    - NetworkX 2.1

### 2.2. 하드웨어
- ~~Turtlebot3 Waffle~~


## 3. 기능
### 3.1. Task planner
- Subscribed Topics
    - robot/state ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
    - robot/pose ([geomegry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Published Topics
    - robot/target ([geomegry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))

### 3.2. Spatial info. manager
- Subscribed Topics
    - map ([nav_msgs/OccupancyGrid](docs.ros.org/api/navi_msgs/html/msg/OccupancyGrid.html))
- Published Topics
    - gvd ([nav_msgs/OccupancyGrid](docs.ros.org/api/navi_msgs/html/msg/OccupancyGrid.html))
    - gvg/marker ([visualization_msgs/MarkerArray](docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
- Services
    - gvg/nearest (shared_control/Nearest), 입력한 위치와 가장 가까운 GVG 노드의 id를 반환한다.
    - gvg/neighbors (shared_control/Neighbors), 입력한 id를 갖는 GVG 노드의 이웃노드 id 리스트를 반환한다.
    - gvg/node (shared_control/Node), 입력한 id를 갖는 노드의 속성을 반환한다.
- Paramters
    - gvd_PM (float, default: 10.0), Origin 사이의 최소거리
    - gvd_BM (float, default: 3.74), GVD에 등록되기 위한 occupied와의 최소거리
    - gvg_minimum_path_distance (float, default: 0.3), GVG 말단이 성립하기 위한 최소거리
    - marker_cycle (float, default: 2.0), gvd와 gvg/marker의 발행 주기
- 개발
    - [ ] GVD에 불필요하게 두텁거나 끊어진 부분이 발생한다.
        - 지도에 따라 다르지만 파라미터를 조정하여 해결할 수 있다.
        - 미세한 부분은 GVG에서 다듬는다.
    - [ ] Unknown을 occupied로 간주한다.
        - 미탐사 지역은 상정하지 않으므로, unknown 격자 주변에서는 GVD가 어지러워진다.
        - 차후 지도의 실시간 갱신기능이 구현되면 unknown도 주행가능한 지역으로 변경한다.
    - [x] 임의의 서브그래프를 GVG로 변환한다.
        - GVD에 포함된 가장 큰 서브그래프를 검색하여 GVG로 변환한다.
    - [x] 규모가 큰 교차로에 2개 이상의 독립된 노드가 발생한다. Cycle이 발생하는 교차로에 결손이 발생한다.
        - ~~Depth first search를 활용하여 교차로를 완성한다.~~
        - ~~탐색위치 전환을 트리거로 인접한 교차로 노드들을 연결한다.~~
        - 동일한 상태를 갖는 노드들을 단일 노드로 압축한다.
    - [x] 이웃노드가 2개인 노드가 발생한다.
        - 불필요한 노드들을 하나씩 제거하고 엣지로 연결한다.
        - ~~스스로 cycle을 만드는 엣지를 제거한다.~~
    - [x] GVG에 불필요한 짧은 말단이 발생한다.
        - 말단을 유지하기 위한 최소거리를 설정한다.
    - [ ] 특정 위치를 GVG에 맵핑할 수 없다.
        - 가장 가까운 노드를 검색하는 기능으로 대신한다.
        - GVG 엣지 데이터를 그래프로 구축해야 한다.

### 3.3. Map server
- Published Topics
    - map ([nav_msgs/OccupancyGrid](docs.ros.org/api/navi_msgs/html/msg/OccupancyGrid.html))

### 3.4. Plan visualizer

### 3.5. Rviz


## 4. 사용법
본 패키지의 커스텀 서비스를 등록하기 위해 컴파일이 필요하다. 작업공간에 `shared_control` 패키지를 위치시키고 다음을 실행한다.
```
$ cd ~/catkin_ws
$ catkin_make
```


## 5. 색인
### 5.1. BCI
Brain-Computer Interface. 본 과제에서는 다음과 같은 방식이 제공된다.
- Motor imagery
    - 이동로봇이 사용자에게 질문하면 약 6초(편차가 크다) 후에 80%의 정확도로 답변을 돌려준다.
    - 현재는 binary question만 가능하다. 3개 이상의 선택지를 질문하려면 각 선택지를 순차적으로 질문해야 한다.
- EEG eye blink
    - 사용자가 눈을 2번 깜빡이면 85%의 정확도로 로봇에게 신호를 전달한다.
- ~~Error-Related Negativity (ERN)~~

### 5.2. GVG
Generalized Voronoi Graph. 지도의 뼈대를 표현한 그래프이다. 이동로봇의 선택지를 최적화하고, BCI에 질문을 요청하는 순간을 결정할 목적으로 활용한다.
