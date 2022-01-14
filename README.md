# Shared Control for Mobile Robots through Navigational Cues
This repository proposes a system for navigational task execution with shared control strategy. This system asks the direction to move when the mobile robot needs user decision. On the contrary, the target pose is updated by receiving the user's answer. Experimental results show that the proposed system is robust to low information transfer rate and user feels less fatigue, therefore this system will be useful to guide the visually impaired or the brain-computer interface user.


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
