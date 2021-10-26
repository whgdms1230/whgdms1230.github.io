---
sort: 3
---

# dwa_local_planner

## 0. 참고 문헌

*- [dwa_local_planner 위키 페이지](http://wiki.ros.org/dwa_local_planner?distro=noetic)*

## 1. dwa_local_planner
<img src="dwa.png"  width="800" height="400">

주어진 글로벌 플래너를 따르기 위한 속도 명령을 생성하는 패키지로, nav_core 패키지의 BaseLocalPlanner 인터페이스를 준수한다.
로컬 플래너는 로봇 주위에 그리드 형태의 가치 맵을 생성하고, 샘플링 타임 동안의 dx,dy,dth를 이용한 시뮬레이션을 통해 가장 높은 점수를 받은 속도 명령(dx,dy,dth)를 내린다.

* dwa 참조 문서
1. [D. Fox, W. Burgard, and S. Thrun. "The dynamic window approach to collision avoidance". The Dynamic Window Approach to local control.](http://www.cs.washington.edu/ai/Mobile_Robotics/postscripts/colli-ieee.ps.gz)
2. [Alonzo Kelly. "An Intelligent Predictive Controller for Autonomous Vehicles". A previous system that takes a similar approach to control.](http://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_7/kelly_alonzo_1994_7.pdf)
3. [Brian P. Gerkey and Kurt Konolige. "Planning and Control in Unstructured Terrain ". Discussion of the Trajectory Rollout algorithm in use on the LAGR robot.](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.330.2120&rep=rep1&type=pdf)

## 2. DWAPlannerROS
dwa_local_planner::DWAPlannerROS는 dwa_local_planner::DWAPlanner의 C++ ROS로 래핑한 개체로, nav_core::BaseLocalPlanner 인터페이스를 기반으로 한다.

## 3. Published Topics
1. ~[name]/global_plan([nav_msgs/Path](http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html)) : local planner가 따르기 위한 global plan 경로, 주로 시각화 목적으로 사용
2. ~[name]/local_plan([nav_msgs/Path](http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html)) : 가장 마지막 사이클에 생성된 local plan 경로, 주로 시각화 목적으로 사용

## 4. Subscribed Topics
1. odom ([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)) : planner에 로봇의 현재 속도를 제공

## 5. Parameters
dwa은 parameter 설정에 따라 커스터마이징 할 수 있다. 자세한 parameter 내용은 [dwa_local_planner 위키 페이지](http://wiki.ros.org/dwa_local_planner?distro=noetic) 및 [Navigation Tunning Guide 위키 페이지](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)를 참조

> [DWAPlannerROS C++ API doc](https://docs.ros.org/en/api/dwa_local_planner/html/classdwa__local__planner_1_1DWAPlannerROS.html)
