---
sort: 1
---

# Navigation

## 0. 참고 문헌
*- [Navigation 위키 페이지](http://wiki.ros.org/navigation/Tutorials/RobotSetup)*

*- [Setup and Configuration of the Navigation Stack on a Robot](http://wiki.ros.org/navigation/Tutorials/RobotSetup)*

*- [Navigation github 페이지](https://github.com/ros-planning/navigation)*

## 1. ROS의 Navigation
2D 기반의 Navigation 스택으로 odometry, 센서 정보, goal pose 등을 기반으로 속도 명령을 통해 모바일 로봇을 구동하는 역할을 한다.

## 2. 하드웨어 요구사항
1. Differential Drive 형태의 로봇 전용
2. Laser 또는 Lidar 등 Mapping과 Localization에 사용될 수 있는 센서
3. 직사각형 또는 원형의 단순한 모형으로 단순화하여 제어되므로, 공간에 비해 로봇의 크기가 크면 제한됨

## 3. Robot Setup
<img src="RobotSetup.png"  width="900" height="400">

위 다이어그램은 Navigation 구성 요소들을 나타낸다. 따라서 Navigation을 운용하기 위해서는 기본적으로 다음을 필요로 한다.

* [ROS](/ROS1/Basic/)
* [tf](/ROS1/Navigation/0002TF.html)
* [Sensor source](/ROS1/Navigation/0003SensorStream.html)([sensor_msgs/LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html) or [sensor_msgs/PointCloud](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud.html))
* [Odometry](/ROS1/Navigation/0004Odometry.html)([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
* [Base Controller](/ROS1/Navigation/0005BaseController.html)([geometry_msgs/Twist](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* [Mapping](/ROS1/Navigation/0006Mapping)

## 4. Navigation Stack Setup
* [PathPlanning](/ROS1/Navigation/01PathPlanning/)
* [Localization](/ROS1/Navigation/02Localization/)