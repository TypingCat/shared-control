# shared_control
본 패키지는 이동로봇의 공유제어를 담당한다. 이동로봇은 그래프를 참고하여 목적지를 결정하며, 그래프에서 교차로가 발생하면 BCI에게 선택을 요청하는 식으로 목적지에 도달한다.

패키지 관리자 노진홍
<br> 인턴, 지능로봇연구단, 한국과학기술연구원
<br> fini@kist.re.kr
<br> 02-958-4864


## 1. 개요
### 1.1. 목표
조이스틱 사용자 대비 이동시간 300% 이내에 무작위 목적지에 도달

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
- 김래현박사님팀
    - 실무자: 김다혜(dahyekim@kist.re.kr), 윤주석(juseok5462@kist.re.kr)
    - 역할: BCI로 사용자의 의도를 획득한다.
- 최종석박사님팀
    - 실무자: 노진홍(fini@kist.re.kr)
    - 역할: BCI-이동로봇을 위한 공유제어를 설계한다.
- 윤상석교수님팀
    - 실무자: 엄홍규(ehg2y@naver.com)
    - 역할: 이동로봇을 제어한다.

![아키텍처](image/architecture.png)

### 1.4. 버전
- `1.0.0` 노드 사이의 프로토콜 확립; 테스트 모듈 구현
- `1.0.1` Task planner 구축
- `1.0.2` Gazebo 연결
- `1.0.3` Interface visualizer 구축


## 2. 개발환경
### 2.1. 소프트웨어
- Ubuntu 16.04
- ROS kinetic
    - Turtlebot3
    - Turtlebot3_msgs
- Python 2.7
    - NetworkX 2.1

### 2.2. 하드웨어
- Turtlebot3 Waffle
    - Gazebo 시뮬레이션


## 3. 기능
### 3.1. Task planner
- Subscribed Topics
    - bci/eyeblink ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), 눈을 깜빡인 횟수
    - robot/state ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), `0`은 정지, `1`은 이동하는 중을 나타낸다.
    - robot/pose ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Published Topics
    - robot/target ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Paramters
    - spin_cycle (float, default: 0.1), 기본적인 단위연산주기
    - planning_cycle (float, default: 0.5), 계획의 단위연산주기
- Expected behaviors
    - Task planner는 4개의 상태: `수면` `대기` `목표노드 도착` `eyeblink`를 갖는 finite-state machine이다.
    - 이동로봇이 목표노드에 도달하거나 eyeblink를 수신하는 이벤트가 발생하면 상태천이를 검토한다.
    - Task planner와 이동로봇의 상태에 따라 선택지를 결정한다.
    - 선택지를 binary question으로 파싱하여 motor imagery에게 전달한다.
    - 획득한 답변을 로봇의 목표자세로 변환하여 출력한다. 이동과 정지 모두 자세로 간주한다.
- Issues
    - [ ] 시작하면 일단 가장 가까운 GVG 노드로 이동한다.
        - 아직은 현재위치가 속한 GVG 엣지를 판단할 수 없다.
        - 차선책으로써 가장 가까운 노드로 이동한다.
    - [ ] 테스트를 위해 `fake_bci` `fake_robot`을 운용한다.
        - 이동로봇의 좌표계는 발행하지 않는다.
        - BCI의 정확도는 반영하지 않는다.
    - [x] Motor imagery 문답 시 다른 기능들이 정지한다.
        - 일회성 타이머로 연산을 분리한다.
        - 이벤트 전후로 다른 연산요청을 처리한다.
        - 상태 갱신과정을 간소화한다.
    - [ ] 가끔 노드의 초기화가 실패한다.
        - 서비스 연결을 실행 후 1초 뒤로 미룬다.
        - 성공할 때까지 초기화를 반복한다.
    - [ ] 가끔 의도하지 않은 노드가 이동목표로 선택된다.
        - GVG 위에서의 현재위치가 명확하지 않다.
        - 중복입력을 방지하기 위해 이벤트 처리 후 일정시간 휴지한다.
    - [x] 트리거와 타이머가 질문을 섞는다.
        - 질문 시퀀스를 분리하여 진행한다. 이 과정이 진행되는 동안 발생하는 이벤트들은 무시한다.
        - 계획 관련 작업을 모두 하나의 함수에서 처리한다.
    - [x] 이동로봇 기준의 시야에서는 선택지가 확인되지 않는다.
        - 선택지를 정면에 놓고 이동 여부를 질문하도록 방침을 변경한다.
    - [ ] 오래 기동하면 state machine이 무너진다.
        - 주로 타이머에 문제가 발생한다.

### 3.2. Spatial information manager
- Subscribed Topics
    - map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/navi_msgs/html/msg/OccupancyGrid.html))
- Published Topics
    - gvg ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
- Services
    - gvg/nearest (shared_control/Nearest), 입력한 위치와 가장 가까운 GVG 노드의 id를 반환한다.
        - 입력: point ([geometry_msgs/Point](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Point.html))
        - 반환: id ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
    - gvg/neighbors (shared_control/Neighbors), 입력한 id를 갖는 GVG 노드의 이웃노드 id 리스트를 반환한다.
        - 입력: id ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
        - 반환: ids\[\] ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
    - gvg/node (shared_control/Node), 입력한 id를 갖는 노드의 속성을 반환한다.
        - 입력: id ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
        - 반환: point ([geometry_msgs/Point](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Point.html))
- Paramters
    - gvd_PM (float, default: 10.0), Origin 사이의 최소거리
    - gvd_BM (float, default: 3.74), GVD에 등록되기 위한 occupied와의 최소거리
    - gvg_minimum_path_distance (float, default: 0.3), GVG 말단이 성립하기 위한 최소거리
- Issues
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
    - [ ] GVG가 불안정하여 가끔 서비스가 실패한다.
        - 초기화 도중 혹은 잦은 서비스 요청에서 발생한다.
        - 예외처리를 추가하여 서비스에 실패했음을 알린다.

### 3.3. Interface visualizer
- Subscribed Topics
    - gvg ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
    - interface/lighter ([geometry_msgs/Point](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Point.html))
    - interface/flicker ([geometry_msgs/Point](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Point.html))
    - interface/douser ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
- Published Topics
    - interface ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
- Parameters
    - publish_cycle (float, default: 1.0), `interface`의 발행 주기
- Issues
    - [x] 아직 인터페이스 규모가 크지 않아 구현하지 않는다.
        - 시각화가 필요한 노드에서 직접 마커를 출력했었으나, 이제 마커를 단일 타이머에서 정리하여 출력한다.
        - 마커의 주기적인 재발행 혹은 정보의 구체적인 이미지화를 돕는다.

### 3.4. Map server
- Published Topics
    - map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/navi_msgs/html/msg/OccupancyGrid.html))

### 3.5. Rviz
- Subscribed Topics
    - map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/navi_msgs/html/msg/OccupancyGrid.html))
    - interface ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
    - camera/rgb/image_raw ([sensor_msgs/Image](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Image.html))
    - (테스트 전용) robot/marker ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
    - (테스트 전용) bci/marker ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
- Issues
    - [ ] 토픽 `robot/marker` `bci/marker`는 노드 `fake_robot` `fake_bci`로부터 발행된다.
        - 즉 해당 토픽들은 테스트를 위한 마커이다. 실제 시스템과 연결할 때에는 해당 패키지에서 마커를 발행하거나, 그에 준하는 정보를 발행해 주어야 한다.

### 3.6. (테스트 전용) Fake BCI
- Subscribed Topics
    - robot/pose ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Published Topics
    - bci/eyeblink ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), 눈을 깜빡인 횟수
- Services
    - bci/motorimagery (shared_control/MotorImagery), 입력받은 리스트(binary question)의 요소 중 하나를 반환한다. 키보드를 인터페이스로 사용한다.
        - 입력: ids\[\] ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
        - 반환: id ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
- Issues
    - [ ] `Fake BCI` 노드를 실제 BCI로 대체해야 한다.

### 3.7. (테스트 전용) Fake robot
- Subscribed Topics
    - robot/target ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Published Topics
    - robot/state ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), `0`은 정지, `1`은 이동하는 중을 나타낸다.
    - robot/pose ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
    - robot/marker ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
- Broadcasted Transformations
    - `map`--`odom`, 이동로봇의 초기위치
    - `odom`--`base_footprint`
- Paramters
    - pos_x (float, default: 0.0), 이동로봇의 x축 좌표 초기값
    - pos_y (float, default: 0.0), 이동로봇의 y축 좌표 초기값
    - pos_th (float, default: 0.0), 이동로봇의 방향각 초기값
    - velocity_lin (float, default: 0.26), 이동로봇의 선속도
    - velocity_ang (float, default: 1.82), 이동로봇의 각속도
    - margin_lin (float, default: 0.1)
    - margin_ang (float, default: 0.1)
    - sim_cycle (float, default: 0.1), 시뮬레이션 연산주기
    - oscillation (float, default: 0.01)
- Issues
    - [ ] `Fake robot` 노드를 실제 이동로봇으로 대체해야 한다.


## 4. 사용법
### 4.1. 설치
본 패키지는 시뮬레이션 대상인 turtlebot3 관련 패키지가 필요하다. 공식 홈페이지의 [PC setup](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/) 파트를 따라 설치하자. 그리고 이하를 따라 추가로 필요한 패키지들을 다운로드한다.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/Taemin0707/shared_control.git
$ git clone https://yssmecha@bitbucket.org/yssmecha/turtlebot3_gazebo.git
$ apt-get install python-pip
$ pip install networkx
$ cd ~/catkin_ws
$ catkin_make
```

### 4.2. 테스트
본 예제는 `shared_control` 패키지의 기능을 확인할 수 있다. 테스트를 위해 이동로봇과 BCI로부터의 메시지를 시뮬레이션한다. 키보드로 BCI 입력을 대체하고 rviz로 진행상황을 출력한다.
```
$ roslaunch shared_control simple_test.launch
```
이동로봇이 그래프를 따라 이동하는 것을 확인할 수 있다.

### 4.3. 이동로봇 시뮬레이터
본 예제는 `4.2`에서 gazebo 시뮬레이터가 추가된 형태이다. 실제 이동로봇을 위한 navigation 기능이 추가된다. 조작법은 동일하다.
```
$ roslaunch shared_control gazebo_test.launch
```
Navigation 성능을 체감할 수 있다.


## 5. 색인
### 5.1. Brain-Computer Interface (BCI)
사용자의 뇌파(electroencephalography, EEG)를 사용하는 인터페이스이다.
- Motor imagery
    - 이동로봇이 사용자에게 질문하면 약 6초(편차가 크다) 후에 80%의 정확도로 답변을 돌려준다.
    - 현재는 binary question만 가능하다. 3개 이상의 선택지를 질문하려면 각 선택지를 순차적으로 질문해야 한다.
- Eye blink
    - 사용자가 눈을 2번 깜빡이면 85%의 정확도로 로봇에게 신호를 전달한다.
- ~~Error-Related Negativity (ERN)~~

### 5.2. Generalized Voronoi Graph (GVG)
지도의 뼈대를 표현한 그래프이다. Brushfire-based AGVD calculation을 사용하여 계산한다.
- 이동로봇의 선택지를 최적화하고, BCI에 질문을 요청하는 순간을 결정할 목적으로 활용한다.
- 현재는 정적인 환경에서만 작동한다.

### 5.3. Navigation
ROS의 `navigation` 스택을 활용하여 이동로봇의 위치를 추정하고 목표로 이동시킨다. 신라대의 `motion_manager`: https://github.com/ZeroAnu/motion_manager 에서 움직임을 관리하고 있다.
