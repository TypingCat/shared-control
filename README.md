# [shared_control](https://github.com/finiel/shared_control)
본 패키지는 이동로봇의 공유제어를 담당한다. 이동로봇은 Generalized Voronoi Graph(GVG)를 참고하여 목적지를 결정하며, GVG에서 교차로가 발생하면 Brain-Computer Interface(BCI)에게 선택을 요청하는 식으로 목적지에 도달한다.

> 패키지 관리자 노진홍
<br> 박사후연구원, 지능로봇연구단, 한국과학기술연구원
<br> finiel@naver.com
<br> 02-958-4864


## 개요
### 목표
조이스틱 사용자 대비 이동시간 ~~300%~~ 200% 이내에 무작위 목적지에 도달

### 방침
1. 로봇은 자신의 위치를 GVG 수준에서 파악할 수 있으며, 인접한 노드들을 이동대상으로 본다.
2. 로봇은 교차로에 도달하면 사용자에게 navigational cue를 제공한다.
3. 사용자는 motorimagery로 경로의 방향을 결정한다.
4. 사용자는 eye blink로 경로선택을 보조하거나 휴면상태를 해제할 수 있다.

### 협업구조
- 김래현박사님팀
    - 실무자: 김다혜(dahyekim@kist.re.kr), 윤주석(juseok5462@kist.re.kr), 권장호(g15007@kist.re.kr), 오승준(ohseungjun@kist.re.kr)
    - 역할: BCI로 사용자의 의도를 획득한다.
- 최종석박사님팀
    - 실무자: 노진홍(fini@kist.re.kr), 최태민(choitm0707@kist.re.kr)
    - 역할: BCI-이동로봇을 위한 공유제어를 설계한다.
- 윤상석교수님팀
    - 실무자: 엄홍규(ehg2y@naver.com)
    - 역할: 이동로봇을 제어한다.

![아키텍처](image/architecture.png)

### 변경점
- `1.0.0` 노드 사이의 프로토콜 확립, `1.0.1` Task planner 구축, `1.0.2` Gazebo 연결, `1.0.3` Interface visualizer 구축, `1.0.4` 좌표계 구축
- `1.1.0` 실제 BCI와 연결, `1.1.1` 평가 모듈 추가, `1.1.2` Joystick 추가, `1.1.3` 그래프 수동작성기능 추가
- `1.2.0` 행동방침 변경, `1.2.1` 실행파일 인자 공유, `1.2.2` Motion manager 인터럽트 기능 추가
- `1.3.0` 이동로봇-BCI 인터페이스 개선, `1.3.1` 지도 확장, `1.3.2` 노드 마운트 초기화, `1.3.3` 로봇의 상태 발행
- `1.4.0` 교차로 대응방식 변경, `1.4.1` Eyeblink 인터페이스 조정, `1.4.2` 경로계획법 튜닝
- `1.5.0` Local planner 변경, `1.5.1` 정지자세 교정, `1.5.2` 주행시야 갱신
- `1.6.0` 1인칭 인터페이스 추가, `1.6.1` 다음 경로의 형태 발생, `1.6.2` 키보드 인터페이스 결합, `1.6.3` 전체화면모드 적용
- `1.7.0` 이동로봇-BCI 인터페이스 개편


## 사용법
### 실행
본 패키지는 두 가지 제어방식과 두 가지 구현방식을 지원한다. 각 방식에 해당하는 실행파일은 다음과 같다.

| | 직접제어 | 공유제어 |
|-|-|-|
| 실제 | ~~dc.launch~~ | sc.launch |
| 시뮬레이션 | dc_gzb.launch | sc_gzb.launch |

### 토픽
interf/cmd/intuit
interf/cmd/assist
interf/nav_cue
interf/robot/motion


- `interf/motorimagery_cue`, 공유제어 --> BCI, Motor imagery가 필요한 시점을 알린다. <br> {header: 메시지 발행시점}
- `interf/motorimagery_result`, BCI --> 공유제어, Motor imagery 결과를 보낸다. <br> {dir: 우(1), 좌(2), 전(3), 후(4), 정지(5)}
- `interf/eyeblink_result`, BCI --> 공유제어, Eye blink 결과를 보낸다. <br> {num: 깜빡임 횟수}
- `interf/robot_state`, 이동로봇의 상태를 보고한다. <br> {motion: 시작하는 움직임}


### Hotspot 네트워크 설정
1. Hotspot을 제공할 컴퓨터에서 Network Connections/Add, Wi-Fi 타입 연결을 생성한다.
2. Wi-Fi 탭에서 Connection name(=Turtlebot3), SSID(=Turtlebot3), Mode(=Hotspot), Device(=wlp1s0)를 설정한다.
3. IPv4 탭에서 Method(=Shared to other computers)를 설정한다. 그리고 저장한다.
4. Create New Wi-Fi Network에서 새로 생성한 네트워크 설정(=Turtlebot3)을 선택한다.
5. Hotspot에 연결할 컴퓨터에서 Connect to Hidden Wi-Fi Network, 위에서 설정한 네트워크(=Turtlebot3)를 찾아 연결한다.

### 조이스틱 연결
XBOX360 조이스틱을 연결하려면 어댑터를 꽂고 패드와 페어링을 한다. 두 기기의 페어링 버튼 `(((`을 동시에 누르면 된다. 그리고 다음을 실행한다. 터미널에 조이스틱 데이터가 출력되면 성공이다.

```
$ sudo rmmod xpad
$ sudo xboxdrv
```


## 설치
### 개발환경
- 소프트웨어
    - Ubuntu 16.04
    - ROS kinetic(with Turtlebot3 package)
    - Python 2.7(with NetworkX 2.1)
- 하드웨어
    - Turtlebot3 Waffle(Gazebo)
    - XBOX360 remote controller
    - Graphic card(nvidia-384)

### 설치 방법
1. Turtlebot3 설치안내 중 [PC setup](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/) 파트를 따라 설치한다.
2. shared_control 외 필요한 패키지들을 설치한다.
    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/finiel/shared_control.git
    $ sudo apt install python-pip xboxdrv ros-kinetic-joy ros-kinetic-teb-local-planner ros-kinetic-realsense-camera
    $ pip install --user networkx==2.1 pygame
    $ cd ~/catkin_ws
    $ catkin_make
    ```


## 노드
### Direct controller
이동로봇의 속도를 직접 제어한다. 키보드와 조이스틱 입력을 지원한다.

- Parameters
    - robot_vel_lin (float, default: 0.6)
    - robot_vel_ang (float, default: 1.82)
    - spin_cycle (float, default: 0.1)

### Evaluator
이동로봇의 움직임을 테스트한다. 무작위 혹은 주어진 목표까지 이동하기까지의 시간을 기록한다.

- Parameters
    - destination_margin (float, default: 0.5), 목적지 반경
    - destination_spawn_x_min (float, default: -5.0), 목적지 무작위 발생영역 (x_min, x_max, y_min, y_max)
    - destination_spawn_x_max (float, default: 5.0)
    - destination_spawn_y_min (float, default: -5.0)
    - destination_spawn_y_max (float, default: 5.0)
    - destination_list_x (float, default: []), 수동 목적지 리스트 (x, y)
    - destination_list_y (float, default: [])
    - spin_cycle (float, default: 0.1)

### Interfacer
간접제어를 위한 프로토콜을 정리한다. 키보드와 BCI 입력을 지원한다.

- Parameters
    - spin_cycle (float, default: 0.1)

### Spatial information manager
공간정보를 처리한다. 주어진 지도를 GVG로 변환하고 검색서비스를 제공한다.

- Paramters
    - gvd_PM (float, default: 10.0), Origin 사이의 최소거리
    - gvd_BM (float, default: 3.74), GVD에 등록되기 위한 occupied와의 최소거리
    - gvg_minimum_path_distance (float, default: 0.3), GVG 말단이 성립하기 위한 최소거리
    - custom_edge_list_x1 (float, default: []), 엣지 (x1, y1, x2, y2)로 구성되는 그래프; 입력하면 해당 그래프가 GVG를 대체
    - custom_edge_list_y1 (float, default: [])
    - custom_edge_list_x2 (float, default: [])
    - custom_edge_list_y2 (float, default: [])

### Task planner
이동로봇의 행동방침을 반영한다. 현재상황을 파악하여 로봇의 이동목표를 결정하거나 질문을 생성한다.

- Paramters
    - spin_cycle (float, default: 0.1)
    - plan_cycle (float, default: 0.5)
    - node_radius (float, default: 2.0), 노드의 반경; cue 발행시점 조절가능
    - robot_vel_lin (float, default: 0.26)
    - robot_vel_ang (float, default: 1.82)
- Expected behaviors
    - Task planner는 3개의 상태(휴면, 준비, 작동)을 기반으로 움직이는 finite-state machine이다. 휴면상태에서 시작한다.
    - 최초노드는 가장 가까운 노드로 설정된다.
    - 이동 중 교차로에 접근하면 cue를 발행한다.
    - 교차로에 도달하면 motor imagery가 선택한 방향으로 회전한다. 회전하는 도중 eye blink가 들어오면 해당 방향으로 이동하고, 아니면 해당방향의 마지막 노드를 향해 이동한다.
    - 말단노드에 도달한다면 휴면상태로 전환한다.

### Visualizer
시각화를 담당한다. 사용자가 상황을 파악할 수 있도록 주행환경과 인터페이스를 마커로 발행한다.

- Parameters
    - MI_marker_len (float, default: 1.0), Motor imagery 선택지를 나타내는 화살표의 길이
    - publish_cycle (float, default: 0.3)
