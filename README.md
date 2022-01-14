# Shared Control for Mobile Robots through Navigational Cues
This repository proposes a system for navigational task execution with shared control strategy. This system asks the direction to move when the mobile robot needs user decision. On the contrary, the target pose is updated by receiving the user's answer. Experimental results show that the proposed system is robust to low information transfer rate and user feels less fatigue, therefore this system will be useful to guide the visually impaired or the brain-computer interface user.

![figure](https://user-images.githubusercontent.com/16618451/149455249-47d7993d-4db7-4fea-ad25-c8ed3ddf60e8.png)

Experiment environment. The actual office(scene 1) was implemented as a simulation model(scene 2). The mobile robot represent the spatial information acquired in this environment by the network(scene 3). This robot is waiting for the user input to move to the target area(green circle).


## Simple Usage
``` bash
roslaunch shared_control minibot.launch
roslaunch shared_control slam.launch robot:=minibot
roslaunch shared_control start.launch
```


## Install
Ubuntu 16.04, ROS kinetic, Turtlebot3

``` bash
cd ~/catkin_ws/src
git clone https://github.com/finiel/shared_control.git
sudo apt install python-pip xboxdrv ros-kinetic-joy ros-kinetic-teb-local-planner ros-kinetic-realsense-camera
pip install --user networkx==2.1 pygame
cd ~/catkin_ws
catkin_make
```
