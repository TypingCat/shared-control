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
    - 실무자: 김다혜(dahyekim@kist.re.kr), 윤주석(juseok5462@kist.re.kr), 권장호(g15007@kist.re.kr), 오승준(ohseungjun@kist.re.kr), 박상인(sipark@kist.re.kr)
    - 역할: BCI로 사용자의 의도를 획득한다.
- 최종석박사님팀
    - 실무자: 노진홍(fini@kist.re.kr)
    - 역할: BCI-이동로봇을 위한 공유제어를 설계한다.

### 변경점
- `1.0.0` 노드 사이의 프로토콜 확립, `1.0.1` Task planner 구축, `1.0.2` Gazebo 연결, `1.0.3` Interface visualizer 구축, `1.0.4` 좌표계 구축
- `1.1.0` 실제 BCI와 연결, `1.1.1` 평가 모듈 추가, `1.1.2` Joystick 추가, `1.1.3` 그래프 수동작성기능 추가
- `1.2.0` 행동방침 변경, `1.2.1` 실행파일 인자 공유, `1.2.2` Motion manager 인터럽트 기능 추가
- `1.3.0` 이동로봇-BCI 인터페이스 개선, `1.3.1` 지도 확장, `1.3.2` 노드 마운트 초기화, `1.3.3` 로봇의 상태 발행
- `1.4.0` 교차로 대응방식 변경, `1.4.1` Eyeblink 인터페이스 조정, `1.4.2` 경로계획법 튜닝
- `1.5.0` Local planner 변경, `1.5.1` 정지자세 교정, `1.5.2` 주행시야 갱신
- `1.6.0` 1인칭 인터페이스 추가, `1.6.1` 다음 경로의 형태 발생, `1.6.2` 키보드 인터페이스 결합, `1.6.3` 전체화면모드 적용
- `1.7.0` 이동로봇-BCI 인터페이스 개편, `1.7.1` 실행주체 분리, `1.7.2` 노드 이동, `1.7.3` 주행정보 기록
- `1.8.0` 단순화 버전 추가


## 사용법
### SLAM
1. 로봇과 서버의 [ROS 네트워크를 설정](http://wiki.ros.org/ROS/NetworkSetup)한다.
2. 로봇을 SLAM 모드로 깨운다.
    ``` bash
    $ roslaunch shared_control slam.launch robot:=minibot   # 미니로봇
    ```
    ``` bash
    $ roslaunch shared_control slam.launch robot:=turtlebot # 터틀봇
    ```
    ``` bash
    $ roslaunch shared_control slam.launch robot:=gazebo    # 가제보
    ```
3. 원격제어 프로그램을 실행한다.
    ``` bash
    $ roslaunch shared_control start.launch share:=false    # 인터페이스
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py # 키보드
    ```
4. 직접 돌아다니며 지도를 작성한 후, 저장한다.
    ``` bash
    $ rosrun map_server map_saver -f 파일이름
    ```

### Navigation
1. 로봇과 서버의 [ROS 네트워크를 설정](http://wiki.ros.org/ROS/NetworkSetup)한다.
2. 실행파일 `navigation.launch`에 지도를 등록한다.
3. 로봇을 Navigation 모드로 깨운다.
    ``` bash
    $ roslaunch shared_control navigation.launch robot:=minibot   # 미니로봇
    ```
    ``` bash
    $ roslaunch shared_control navigation.launch robot:=turtlebot # 터틀봇
    ```
    ``` bash
    $ roslaunch shared_control simulation.launch                  # 가제보
    ```
4. 원격제어 프로그램을 실행한다. 필요하다면 실행파일 `start.launch`에서 파라미터를 수정한다.
    ``` bash
    $ roslaunch shared_control start.launch share:=true           # 인터페이스
    ```

### Simple
1. 로봇과 서버의 [ROS 네트워크를 설정](http://wiki.ros.org/ROS/NetworkSetup)한다.
2. 로봇을 깨운다.
    ``` bash
    $ roslaunch shared_control minibot.launch
    ```
3. 단순화된 원격제어 프로그램을 실행한다.
    ``` bash
    $ roslaunch shared_control simple.launch
    ```

### 자주 발생하는 문제와 해결방법
- 로봇이 위치추정을 실패, 특히 회전시의 변화를 따라가지 못하는 경우
    - IMU 재시작
        ``` bash
        $ rostopic pub reset std_msgs/Empty
        ```
- 현재위치가 지도상의 위치와 다를 경우
    - Rviz에서 `2D Pose Estimation`으로 지도상에 자세 입력
    - 이후 로봇을 회전시켜 위치 미세조정
        ``` bash
        $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
        ```

### Subscribed topic for BCI
- `interf/cmd/intuit`: 로봇에게 이동방향을 지시한다.
    - `header`
    - `dir` = {1: 오른쪽, 2: 왼쪽, 3: 앞}
- `interf/cmd/assist`: 로봇에게 행동시점을 지시한다.
    - `header`
    - `num` = {2: 교차로에서 회전정지시점, 혹은 말단노드 탈출시점}

### Published topic for BCI
- `interf/nav_cue`: 도달예정인 교차로의 정보를 제공한다.
    - `header`
    - `dist` = 교차로와의 거리
    - `right` = 오른쪽 경로의 수
    - `left` = 왼쪽 경로의 수
    - `forward` = {0: 전방경로 없음, 1: 전방경로 있음}
    - `backward` = {0: 후방경로 없음, 1: 후방경로 있음}
- `interf/robot/motion`: 로봇이 선택한 움직임을 알린다.
    - `header`
    - `motion` = {1: 오른쪽, 2: 왼쪽, 3: 앞, 4: 뒤, 5: 정지, 6: 알림, 7: 이동}

### Hotspot 네트워크 설정
1. Hotspot을 제공할 컴퓨터에서 Network Connections/Add, Wi-Fi 타입 연결을 생성한다.
2. Wi-Fi 탭에서 Connection name(=Turtlebot3), SSID(=Turtlebot3), Mode(=Hotspot), Device(=wlp1s0)를 설정한다.
3. IPv4 탭에서 Method(=Shared to other computers)를 설정한다. 그리고 저장한다.
4. Create New Wi-Fi Network에서 새로 생성한 네트워크 설정(=Turtlebot3)을 선택한다.
5. Hotspot에 연결할 컴퓨터에서 Connect to Hidden Wi-Fi Network, 위에서 설정한 네트워크(=Turtlebot3)를 찾아 연결한다.

### Joystick 연결
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

### 설치 순서
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
