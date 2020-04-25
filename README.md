# complier

# Turtlebot3 Burger



# manual

http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview



# book

- [ROS Robot Programming (English)](http://www.robotis.com/service/download.php?no=719)
http://www.robotis.com/service/download.php?no=719

- [ROS机器人编程 (中文)](http://www.robotis.com/service/download.php?no=720)
http://www.robotis.com/service/download.php?no=720



**[Contents]**

- Chapter 01 Robot Software Platform
- Chapter 02 Robot Operating System ROS
- Chapter 03 Configuring the ROS Development Environment
- Chapter 04 Important Concepts of ROS
- Chapter 05 ROS Commands
- Chapter 06 ROS Tools
- Chapter 07 Basic ROS Programming
- Chapter 08 Robot/Sensor/Motor
- Chapter 09 Embedded System
- Chapter 10 Mobile Robots
- Chapter 11 SLAM and Navigation
- Chapter 12 Service Robot
- Chapter 13 Manipulator





# Source code



download code

```
cd ~/catkin_ws/src

git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
```



```

➜  src tree -L 2
.
├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
├── turtlebot3
│   ├── ISSUE_TEMPLATE.md
│   ├── LICENSE
│   ├── README.md
│   ├── turtlebot3
│   ├── turtlebot3_bringup
│   ├── turtlebot3_description
│   ├── turtlebot3_example
│   ├── turtlebot3_navigation
│   ├── turtlebot3_slam
│   └── turtlebot3_teleop
├── turtlebot3_msgs
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   ├── LICENSE
│   ├── msg
│   ├── package.xml
│   └── README.md
└── turtlebot3_simulations
    ├── LICENSE
    ├── README.md
    ├── turtlebot3_fake
    ├── turtlebot3_gazebo
    └── turtlebot3_simulations

14 directories, 11 files
```



build

```
cd ~/catkin_ws
catkin_make
```



# Simulation

start ros master

```
roscore &
```



setup robot type

```
export TURTLEBOT3_MODEL=burger
```



gazebo server

```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```



Test

```
roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
```



# Tele operation

http://emanual.robotis.com/docs/en/platform/turtlebot3/teleoperation/#teleoperation



keyboard

```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```





# SLAM

http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/



- gmapping
  - [ROS WIKI](http://wiki.ros.org/gmapping), [Github](https://github.com/ros-perception/slam_gmapping)
- cartographer
  - [ROS WIKI](http://wiki.ros.org/cartographer), [Github](https://github.com/googlecartographer/cartographer)
- hector
  - [ROS WIKI](http://wiki.ros.org/hector_slam), [Github](https://github.com/tu-darmstadt-ros-pkg/hector_slam)
- karto
  - [ROS WIKI](http://wiki.ros.org/slam_karto), [Github](https://github.com/ros-perception/slam_karto)
- frontier_exploration
  - [ROS WIKI](http://wiki.ros.org/frontier_exploration), [Github](https://github.com/paulbovbel/frontier_exploration)



## CODE

Gmapping slam code 

```
git clone https://github.com/ros-perception/slam_gmapping.git
git clone https://github.com/ros-perception/openslam_gmapping.git
```



hector

```
https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
```





## RUN

gmapping slam

```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```



```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector 
```



## Save MAP



```
rosrun map_server map_saver -f ~/map
```



**white** is the free area in which the robot can move, **black** is the occupied area in which the robot can not move, and **gray** is the unknown area.



# Navigation

download code

```
git clone https://github.com/ros-planning/navigation.git
```



```
git clone https://github.com/ros/geometry2.git 
```



run

```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

```





# opencr



arduino



https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_burger/turtlebot3_core/turtlebot3_core.ino

