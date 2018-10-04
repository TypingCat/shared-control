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
- 김래현박사님팀
    - 실무자: 김다혜, dahyekim@kist.re.kr
    - 역할: BCI로 사용자의 의도를 획득한다.
- 최종석박사님팀
    - 실무자: 노진홍, fini@kist.re.kr
    - 역할: BCI-이동로봇을 위한 공유제어를 설계한다.
- 윤상석교수님팀
    - 실무자: 엄홍규, ehg2y@naver.com
    - 역할: 이동로봇을 제어한다.

![아키텍처](image/architecture.png)


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
    - bci/eyeblink ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), 눈을 깜빡인 횟수
    - robot/state ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), `0`은 정지, `1`은 이동하는 중을 나타낸다.
    - robot/pose ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Published Topics
    - robot/target ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Paramters
    - spin_cycle (float, default: 0.1), 기본적인 단위연산주기
    - planning_cycle (float, default: 0.5), 계획의 단위연산주기
    - goal_margin (float, default: 0.01), 목표영역의 반경
- Issues
    - [ ] 시작하면 일단 가장 가까운 GVG 노드로 이동한다.
        - 아직은 현재 위치가 속한 GVG 엣지를 판단할 수 없다. 차선책으로, 가장 가까운 노드로 이동한다.
    - [ ] 테스트를 위해 `fake_bci`, `fake_robot`을 운용한다.
        - 이동로봇의 좌표계는 발행하지 않는다.
        - BCI의 정확도는 반영하지 않는다. 명령을 연속으로 입력할 경우 가끔 의도하지 않은 방향으로 이동한다.
    - [ ] 노드 초기화가 실패한다.
        - 초기화를 실행 후 1초(임의) 뒤로 미룬다. 미봉책이다.
    - [ ] 트리거와 타이머가 질문을 섞는다.
        - 질문 시퀀스를 분리하여 lock을 설정한다. 질문 도중 들어오는 트리거는 무시된다.
        - 두 작업을 모두 하나의 함수에서 처리한다. 해당 함수에서 로봇의 상태와 위치를 관리한다.

### 3.2. Spatial info. manager
- Subscribed Topics
    - map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/navi_msgs/html/msg/OccupancyGrid.html))
- Published Topics
    - gvd ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/navi_msgs/html/msg/OccupancyGrid.html)), GVD를 점유된 격자로 표현한다.
    - gvg/marker ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
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
    - marker_cycle (float, default: 2.0), `gvd`와 `gvg/marker`의 발행 주기
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
        - 일단 예외처리를 추가하여 서비스에 실패했음을 알린다.

### 3.3. Plan visualizer
- Issues
    - [ ] 아직 인터페이스 규모가 크지 않아 구현하지 않는다.
        - 시각화가 필요한 노드에서 직접 마커를 출력한다.

### 3.4. Map server
- Published Topics
    - map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/navi_msgs/html/msg/OccupancyGrid.html))

### 3.5. Rviz
- Subscribed Topics
    - map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/navi_msgs/html/msg/OccupancyGrid.html))
    - gvg/marker ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
    - robot/marker ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
    - bci/marker ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
- Issues
    - [ ] 토픽 `robot/marker`, `bci/marker`는 노드 `fake_robot`, `fake_bci`로부터 발행된다.
        - 즉 해당 토픽들은 테스트를 위한 마커이다. 실제 시스템과 연결할 때에는 해당 패키지에서 마커를 발행하거나, 그에 준하는 정보를 발행해 주어야 한다.

### 3.6. (테스트 전용) Fake BCI
- Subscribed Topics
    - robot/pose ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Published Topics
    - bci/eyeblink ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), 눈을 깜빡인 횟수
    - bci/marker ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
- Services
    - bci/motorimagery (shared_control/MotorImagery), 입력받은 리스트(binary question)의 요소 중 하나를 반환한다. 키보드를 인터페이스로 사용한다.
        - 입력: ids\[\] ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))
        - 반환: id ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html))

### 3.7. (테스트 전용) Fake robot
- Subscribed Topics
    - robot/target ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
- Published Topics
    - robot/state ([std_msgs/Int32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int32.html)), `0`은 정지, `1`은 이동하는 중을 나타낸다.
    - robot/pose ([geometry_msgs/Pose](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html))
    - robot/marker ([visualization_msgs/MarkerArray](http://docs.ros.org/api/navi_msgs/html/msg/MarkerArray.html))
- Paramters
    - sim_cycle (float, default: 0.1), 시뮬레이션 연산주기
    - robot_x (float, default: 0.0), 이동로봇의 x축 좌표 초기값
    - robot_y (float, default: 0.0), 이동로봇의 y축 좌표 초기값
    - robot_th (float, default: 0.0), 이동로봇의 방향각 초기값
    - robot_velocity (float, default: 0.2), 이동로봇의 속도
    - goal_margin (float, default: 0.01), 목표영역의 반경


## 4. 사용법
본 패키지의 커스텀 서비스를 등록하기 위해 컴파일이 필요하다. 작업공간 `~/catkin_ws`의 `src` 폴더에 `shared_control` 패키지를 위치시키고 다음을 실행한다. 이 과정은 최초 설치시에만 실행하면 된다.
```
$ cd ~/catkin_ws
$ catkin_make
```
아키텍처가 계획대로 구현될 경우 `shared_control/launch/KES.launch`를 사용하여 다른 패키지들과 함께 실행하면 된다.

테스트를 하려면 이하와 같이 `shared_control/launch/KES_test.launch`를 실행한다: 여기에는 테스트를 위해 이동로봇과 BCI에서 제공해야 할 메시지들을 임의로 생성하는 노드들이 포함되어 있으며, 지도는 `shared_control/map`에서 불러온다. Eyeblink 신호를 보내려면 `w`를, motorimagery 신호를 보내려면 `a`와 `d`를 사용한다.
```
$ roslaunch shared_control KES_test.launch
```

위의 명령으로는 터미널에서만 작동하지만 Rviz로 진행상황을 시각화할 수도 있다. 다른 터미널에서 Rviz를 실행하고, `Rviz/File/Open Config`에서 환경설정파일 `shared_control/launch/KES.rviz`를 선택하자. 로봇의 자세와 목표는 각각 적색 화살표와 녹색 화살표로 표현되며, 적색 점으로 구성된 그래프 `GVG` 위에서 이동하는 것을 확인할 수 있다. 사용자의 선택이 필요할 경우 해당 선택으로 이동할 수 있는 위치가 적색 원 `a`와 청색 원 `d`로 표현된다.
```
$ rviz
```


## 5. 색인
### 5.1. BCI
Brain-Computer Interface. 본 과제에서는 다음과 같은 방식이 제공된다.
- Motor imagery
    - 이동로봇이 사용자에게 질문하면 약 6초(편차가 크다) 후에 80%의 정확도로 답변을 돌려준다.
    - 현재는 binary question만 가능하다. 3개 이상의 선택지를 질문하려면 각 선택지를 순차적으로 질문해야 한다.
- Eye blink
    - 사용자가 눈을 2번 깜빡이면 85%의 정확도로 로봇에게 신호를 전달한다.
- ~~Error-Related Negativity (ERN)~~

### 5.2. GVG
Generalized Voronoi Graph. 지도의 뼈대를 표현한 그래프이다. 이동로봇의 선택지를 최적화하고, BCI에 질문을 요청하는 순간을 결정할 목적으로 활용한다.
