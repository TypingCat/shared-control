# shared_control
본 패키지는 이동로봇의 공유제어를 담당한다. 이동로봇은 그래프를 참고하여 목적지를 결정하며, 그래프에서 교차로가 발생하면 BCI에게 선택을 요청하는 식으로 목적지에 도달한다.

1. [개요](#개요)
    + [목표](#목표)
    + [방침](#방침)
    + [협업구조](#협업구조)
    + [변경점](#변경점)
2. [개발환경](#개발환경)
    + [소프트웨어](#소프트웨어)
    + [하드웨어](#하드웨어)
3. [기능](#기능)
    + [Task planner](#task-planner)
    + [Spatial information manager](#spatial-information-manager)
    + [Interface visualizer](#interface-visualizer)
    + [Evaluator](#evaluator)
    + [BCI](#bci)
    + [Fake robot (position)](#fake-robot-(position))
    + [Fake robot (velocity)](#fake-robot-(velocity))
4. [사용법](#사용법)
    + [설치](#설치)
    + [조이스틱 연결](#조이스틱-연결)
    + [실행](#실행)
5. [색인](#색인)
    + [Brain-Computer Interface (BCI)](#brain-computer-interface-(bci))
    + [Generalized Voronoi Graph (GVG)](#generalized-voronoi-graph-(gvg))

> 패키지 관리자 노진홍
<br> 인턴, 지능로봇연구단, 한국과학기술연구원
<br> fini@kist.re.kr
<br> 02-958-4864


## 개요
### 목표
조이스틱 사용자 대비 이동시간 ~~300%~~ 200% 이내에 무작위 목적지에 도달

### 방침
1. 로봇에게는 지도가 주어진다. 로봇은 자신의 위치를 GVG 수준에서 파악할 수 있으며, 현재 위치와 인접한 노드들을 이동대상으로 본다.
2. 로봇은 교차로에 도달하면 motor imagery를 통해 사용자에게 이동방향을 질문한다.
3. 로봇은 막다른 길에 다다르면 대기한다.
4. 사용자는 eye blink로 선택을 번복하거나 대기상태를 해제할 수 있다.

### 협업구조
- 김래현박사님팀
    + 실무자: 김다혜(dahyekim@kist.re.kr), 윤주석(juseok5462@kist.re.kr), 권장호(g15007@kist.re.kr)
    + 역할: BCI로 사용자의 의도를 획득한다.
- 최종석박사님팀
    + 실무자: 노진홍(fini@kist.re.kr), 최태민(choitm0707@kist.re.kr)
    + 역할: BCI-이동로봇을 위한 공유제어를 설계한다.
- 윤상석교수님팀
    + 실무자: 엄홍규(ehg2y@naver.com)
    + 역할: 이동로봇을 제어한다.

![아키텍처](image/architecture.png)

### 변경점
- `1.0.0` 노드 사이의 프로토콜 확립; 테스트 모듈 구현
    + `1.0.1` Task planner 구축
    + `1.0.2` Gazebo 연결
    + `1.0.3` Interface visualizer 구축
    + `1.0.4` 이동로봇 좌표계 구축
- `1.1.0` 실제 BCI 연결 시도
    + `1.1.1` 평가 모듈 추가
    + `1.1.2` Joystick 추가
    + `1.1.3` 그래프 수동작성기능 추가
- `1.2.0` 행동방침 변경


## 개발환경
### 소프트웨어
- Ubuntu 16.04
- ROS kinetic
    + Turtlebot3
    + Turtlebot3_msgs
- Python 2.7
    + NetworkX 2.1

### 하드웨어
- Turtlebot3 Waffle
    + Gazebo 시뮬레이션
- XBOX360 무선 컨트롤러


## 기능
### Task planner
이동로봇의 행동방침을 결정한다. 현재 상황을 파악하여 로봇의 이동목표를 결정하거나 질문을 생성한다.
- Subscribed Topics
    + bci/eyeblink ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), 눈을 깜빡인 횟수
    + robot/state ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), `0`은 정지, `1`은 이동하는 중을 나타낸다.
    + robot/pose ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Published Topics
    + robot/target ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
    + interface/douser ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
    + interface/MID_L (shared_control/MID), Motor imagery 선택지를 나타내는 왼쪽 화살표 메타정보
    + interface/MID_R (shared_control/MID), Motor imagery 선택지를 나타내는 오른쪽 화살표 메타정보
    + interface/MID_confirm (shared_control/MID), Motor imagery가 선택한 결과를 나타내는 화살표 메타정보
- Paramters
    + spin_cycle (float, default: 0.1), 연산주기
- Expected behaviors
    + Task planner는 4개의 상태(휴면, 대기, 계획, 이동)을 갖는 finite-state machine이다.
    + 이동로봇이 목표노드에 도달하거나 eye blink를 수신하는 이벤트가 발생하면 다음 이동목표를 검토한다.
    + 이동로봇의 상태와 GVG상의 위치에 따라 사용자의 선택지를 좁힌다. 선택할 필요가 없는 경우에는 질문할 필요 없이 이동하거나 대기한다.
    + 선택지를 binary question으로 변환하여 사용자에게 motor imagery로 질문한다.
    + 획득한 답변을 로봇의 목표자세로 변환하여 출력한다.
- Issues
    + [ ] 시작하면 일단 가장 가까운 GVG 노드로 이동한다.
        - 아직은 현재위치가 속한 GVG 엣지를 판단할 수 없다.
        - 차선책으로 초기화되면 가장 가까운 노드로 이동한다.
    + [x] 테스트를 위해 `fake_bci` `fake_robot`을 운용한다.
        - ~~이동로봇의 좌표계는 발행하지 않는다.~~
        - 이동로봇의 좌표계를 방송한다.
        - BCI의 정확도는 반영하지 않는다.
    + [x] Motor imagery 문답 시 다른 기능들이 정지한다.
        - 이벤트 전후로 다른 연산요청을 처리한다.
        - 상태 갱신과정을 간소화한다.
    + [x] 노드의 초기화가 실패한다.
        - 서비스 연결을 실행 후 1초 뒤로 미룬다.
        - 성공할 때까지 초기화를 반복한다.
    + [x] 의도하지 않은 노드가 이동목표로 선택된다.
        - GVG 위에서의 현재위치가 명확하지 않다.
        - 중복입력을 방지하기 위해 이벤트 처리 후 일정시간 휴지한다.
    + [x] 트리거와 타이머가 질문을 섞는다.
        - 질문 시퀀스를 분리하여 진행한다. 이 과정이 진행되는 동안 발생하는 이벤트들은 무시한다.
        - 계획 관련 작업을 모두 하나의 함수에서 처리한다.
    + [x] 이동로봇 기준의 시야에서는 선택지가 확인되지 않는다.
        - 선택지를 정면에 놓고 이동 여부를 질문하도록 방침을 변경한다.
    + [x] 오래 기동하면 state machine이 무너진다.
        - 다른 패키지와의 타이머가 엇갈리며 문제가 발생한다.
        - ~~뒤늦게 갱신되는 상태로 인해 노드도달여부 인지가 불안정했었다. 노드도달여부를 몇차례 연속으로 확인함으로써 상태천이과정을 안정화시켰다.~~
        - 계획 상태를 추가하고 상태천이의 조건을 좁힌다.
        - 계획에 한정하여 일회성 타이머로 상태천이를 처리한다.
    + [ ] 교차로가 3갈래 이하인 그래프에서만 작동한다.
        - Motor imagery가 binary question만 처리할 수 있는 상황에 맞추어 행동방침을 변경하였다.
        - 현재는 4갈래 이상의 교차로가 발생하면 대기한다.
        - 현재는 말단노드에서만 출발이 가능하다.

### Spatial information manager
공간정보를 처리한다. 주어진 지도를 GVG로 변환하고 검색서비스를 제공한다.
- Subscribed Topics
    + map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/navi_msgs/html/msg/OccupancyGrid.html))
- Published Topics
    + interface/graph ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
- Services
    + gvg/nearest (shared_control/Nearest), 입력한 위치와 가장 가까운 GVG 노드의 id를 반환한다.
        - 입력: point ([geometry_msgs/Point](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Point.html))
        - 반환: id ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
    + gvg/neighbors (shared_control/Neighbors), 입력한 id를 갖는 GVG 노드의 이웃노드 id 리스트를 반환한다.
        - 입력: id ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
        - 반환: ids\[\] ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
    + gvg/node (shared_control/Node), 입력한 id를 갖는 노드의 속성을 반환한다.
        - 입력: id ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
        - 반환: point ([geometry_msgs/Point](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Point.html))
- Paramters
    + gvd_PM (float, default: 10.0), Origin 사이의 최소거리
    + gvd_BM (float, default: 3.74), GVD에 등록되기 위한 occupied와의 최소거리
    + gvg_minimum_path_distance (float, default: 0.3), GVG 말단이 성립하기 위한 최소거리
    + custom_edge_list_x1 (float, default: []), GVG 대신 임의로 (x1, y1)--(x2, y2) 형태의 엣지를 생성한다. 센티미터 단위로 입력받는다.
    + custom_edge_list_y1 (float, default: [])
    + custom_edge_list_x2 (float, default: [])
    + custom_edge_list_y2 (float, default: [])
- Issues
    + [x] GVD에 불필요하게 두텁거나 끊어진 부분이 발생한다.
        - 지도에 따라 다르지만 파라미터를 조정하여 해결할 수 있다.
        - 미세한 부분은 GVG에서 다듬는다.
    + [ ] Unknown을 occupied로 간주한다.
        - 미탐사 지역은 상정하지 않으므로, unknown 격자 주변에서는 GVD가 어지러워진다.
        - 차후 지도의 실시간 갱신기능이 구현되면 unknown도 주행가능한 지역으로 변경한다.
    + [x] 임의의 서브그래프를 GVG로 변환한다.
        - GVD에 포함된 가장 큰 서브그래프를 검색하여 GVG로 변환한다.
    + [x] 규모가 큰 교차로에 2개 이상의 독립된 노드가 발생한다. Cycle이 발생하는 교차로에 결손이 발생한다.
        - ~~Depth first search를 활용하여 교차로를 완성한다.~~
        - ~~탐색위치 전환을 트리거로 인접한 교차로 노드들을 연결한다.~~
        - 동일한 상태를 갖는 노드들을 단일 노드로 압축한다.
    + [x] 이웃노드가 2개인 노드가 발생한다.
        - 불필요한 노드들을 하나씩 제거하고 엣지로 연결한다.
        - ~~스스로 cycle을 만드는 엣지를 제거한다.~~
    + [x] GVG에 불필요한 짧은 말단이 발생한다.
        - 말단을 유지하기 위한 최소거리를 설정한다.
    + [ ] 특정 위치를 GVG에 맵핑할 수 없다.
        - 가장 가까운 노드를 검색하는 기능으로 대신한다.
        - GVG 엣지 데이터를 그래프로 구축해야 한다.
    + [x] GVG가 불안정하여 가끔 서비스가 실패한다.
        - 초기화 도중 혹은 잦은 서비스 요청에서 발생한다.
        - 예외처리를 추가하여 서비스에 실패했음을 알린다.
    + [x] 시연을 위해 GVG가 아닌 임의의 그래프 생성기능이 필요하다.

### Interface visualizer
시각화를 담당한다. 사용자가 상황을 파악할 수 있도록 주행환경과 인터페이스를 마커로 발행한다.
- Subscribed Topics
    + interface/douser ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
    + interface/MID_L (shared_control/MID), Motor imagery 선택지를 나타내는 왼쪽 화살표 메타정보
    + interface/MID_R (shared_control/MID), Motor imagery 선택지를 나타내는 오른쪽 화살표 메타정보
    + interface/MID_confirm (shared_control/MID), Motor imagery가 선택한 결과를 나타내는 화살표 메타정보
    + interface/destination ([geometry_msgs/Point](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Point.html))
    + robot/pose ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Published Topics
    + interface ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
- Parameters
    + publish_cycle (float, default: 0.3), `interface`의 발행 주기
    + MI_marker_len (float, default: 1.0), Motor imagery 선택지를 나타내는 화살표의 길이
- Issues
    + [x] 아직 인터페이스 규모가 크지 않아 구현하지 않는다.
        - 시각화가 필요한 노드에서 직접 마커를 출력했었으나, 이제 마커를 단일 타이머에서 정리하여 출력한다.
        - 마커의 주기적인 재발행 혹은 정보의 구체적인 이미지화를 돕는다.

### Evaluator
이동로봇의 움직임을 테스트한다. 무작위 혹은 주어진 목표까지 이동하기까지의 시간을 기록한다.
- Subscribed Topics
    + map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/navi_msgs/html/msg/OccupancyGrid.html))
    + robot/state ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
    + robot/pose ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Published Topics
    + interface/destination ([geometry_msgs/Point](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Point.html))

### BCI
실제 BCI와 통신한다. Motor imagery와 eye blink 기능을 제공한다.
- Subscribed Topics
    + robot/pose ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Published Topics
    + bci/eyeblink ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), 눈을 깜빡인 횟수
- Services
    + bci/motorimagery (shared_control/MotorImagery), 입력받은 리스트(binary question)의 요소 중 하나를 반환한다. 키보드를 인터페이스로 사용한다.
        - 입력: ids\[\] ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
        - 반환: id ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
- Issues
    + [x] `Fake BCI` 노드를 실제 BCI로 대체해야 한다.

### Fake robot (position)
이동로봇이 목표자세를 향해 이동하는 것처럼 시뮬레이션한다. 주행환경을 무시한다.
- Subscribed Topics
    + robot/target ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Published Topics
    + robot/state ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), `0`은 정지, `1`은 이동하는 중을 나타낸다.
    + robot/pose ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Broadcasted Transformations
    + `map`~`odom`, 이동로봇의 초기위치
    + `odom`~`base_footprint`
- Paramters
    + pose_x (float, default: 0.0), 이동로봇의 x축 좌표 초기값
    + pose_y (float, default: 0.0), 이동로봇의 y축 좌표 초기값
    + pose_th (float, default: 0.0), 이동로봇의 방향각 초기값
    + velocity_lin (float, default: 0.26), 이동로봇의 선속도
    + velocity_ang (float, default: 1.82), 이동로봇의 각속도
    + margin_lin (float, default: 0.1)
    + margin_ang (float, default: 0.1)
    + sim_cycle (float, default: 0.1), 시뮬레이션 연산주기
    + oscillation (float, default: 0.01)
- Issues
    + [x] 좌표계가 발행되지 않아 rviz에서 이동로봇을 정상적으로 출력하지 못한다.
        - 직접 좌표계를 계산하여 발행한다.
    + [ ] 움직임이 불안정하다. 파라미터를 어떻게 설정했느냐에 따라 목표에 수렴하지 못하거나 진동할 수도 있다.

### Fake robot (velocity)
이동로봇이 목표속도를 따라 이동하는 것처럼 시뮬레이션한다. 주행환경을 무시한다.
- Subscribed Topics
    + cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Twist.html))
- Published Topics
    + robot/state ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), `0`은 정지, `1`은 이동하는 중을 나타낸다.
    + robot/pose ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Broadcasted Transformations
    + `map`~`odom`, 이동로봇의 초기위치
    + `odom`~`base_footprint`
- Paramters
    + pose_x (float, default: 0.0), 이동로봇의 x축 좌표 초기값
    + pose_y (float, default: 0.0), 이동로봇의 y축 좌표 초기값
    + pose_th (float, default: 0.0), 이동로봇의 방향각 초기값
    + velocity_lin (float, default: 0.26), 이동로봇의 선속도
    + velocity_ang (float, default: 1.82), 이동로봇의 각속도
    + sim_cycle (float, default: 0.1), 시뮬레이션 연산주기


## 사용법
### 설치
본 패키지는 시뮬레이션 대상인 turtlebot3 관련 패키지가 필요하다. 공식 홈페이지의 [PC setup](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/) 파트를 따라 설치하자. 그리고 이하를 따라 추가로 필요한 패키지들을 설치한다.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/finiel/shared_control.git
$ git clone https://yssmecha@bitbucket.org/yssmecha/turtlebot3_gazebo.git
$ git clone https://github.com/ZeroAnu/motion_manager.git
$ sudo apt-get install python-pip xboxdrv ros-kinetic-joy
$ pip install networkx
$ cd ~/catkin_ws
$ catkin_make
```

### 조이스틱 연결
XBOX360 조이스틱을 연결하려면 어댑터를 꽂고 패드와 페어링을 한다. 두 기기의 페어링 버튼 `(((`을 동시에 누르면 된다. 그리고 다음을 실행한다. 터미널에 조이스틱 데이터가 출력되면 성공이다.
```
$ sudo rmmod xpad
$ sudo xboxdrv
```

### 실행
본 패키지는 입력과 출력에 따른 실행방법들을 제공한다. 파라미터는 해당 launch 파일에서 수정할 수 있다.

| 입력 \ 출력 | 시뮬레이션 | Gazebo |
| :-: | :-: | :-: |
| 키보드 | \$ roslaunch shared_control key_sim.launch | \$ roslaunch shared_control key_gzb.launch |
| 조이스틱 | \$ roslaunch shared_control joy_sim.launch | \$ roslaunch shared_control joy_gzb.launch |
| BCI | \$ roslaunch shared_control bci_sim.launch | |

- 입력
    - 키보드: BCI를 대체하는 인터페이스이다. 키 `a`와 `d`가 motor imagery, 키 `s` 가 eye blink를 대체한다.
    - 조이스틱: XBOX360 조이스틱으로 로봇의 속도를 직접 제어할 수 있다.
    - BCI: Brain-Computer Interface.
- 출력
    - 시뮬레이션: 단순한 수식으로 로봇의 좌표만 계산한다.
    - Gazebo: 로봇과 주행환경을 시뮬레이션한다. 실제 로봇과 동일한 메시지를 다룬다.


## 색인
### Brain-Computer Interface (BCI)
사용자의 뇌파(electroencephalography, EEG)를 사용하는 인터페이스이다.
- Motor imagery
    + 사용자에게 질문하고 답변을 뇌파로 인식한다.
    + 현재는 binary question만 가능하다.
- Eye blink
    + 사용자의 눈 깜빡임을 뇌파로 인식한다.
- ~~Error-Related Negativity (ERN)~~

### Generalized Voronoi Graph (GVG)
지도의 뼈대를 표현한 그래프이다. Brushfire-based AGVD calculation을 사용하여 계산한다. 이동로봇의 선택지를 최적화하고 BCI에 질문을 요청하는 순간을 결정할 목적으로 활용한다.
