---
sort: 2
---

# move_base

## 0. 참고 문헌

*- [move_base 위키 페이지](http://wiki.ros.org/move_base)*

## 1. move_base
<img src="move_base.png"  width="900" height="400">

move_base는 로봇의 base를 최종 목적지로 이동시키기 위한 패키지이다. move_base는 글로벌 및 로컬 플래너를 이용하여 네비게이션 task를 수행한다. nav_core 인터페이스를 준수하는 모든 글로벌/로컬 플래너를 사용할 수 있으며, 글로벌/로컬 플래너에 필요한 각각의 costmap을 사용한다.

## 2. Action API
move_base 노드는 geometry_msgs/PoseStamped를 포함하는 goals을 가져오는 SimpleActionServer의 구현을 제공한다. 따라서, 사용자는 상태를 추적하기 위해 SimpleActionClient를 통해 move_base에 goals를 보내는 것을 추천한다. 

> [actionlib doc](http://wiki.ros.org/actionlib)

* Action Subscribed Topics
1. move_base/goal(move_base_msgs/MoveBaseActionGoal) : move_base 내에서 goal 요청
2. move_base/cancel(actionlib_msgs/GoalID) : 특정 goal를 취소 요청

* Action Published Topics
1. move_base/feedback(move_base_msgs/MoveBaseActionFeedback) : 로봇 base의 현재 위치 피드백
2. move_base/status(actionlib_msgs/GoalStatusArray) : 현재 Task에 대한 상태 정보 제공
3. move_base/result(move_base_msgs/MoveBaseActionResult) : move_base 액션에 대한 결과를 비움

## 3. Subscriped Topics
1. move_base_simple/goal([geometry_msgs/PoseStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)) : 사용자가 move_base에 내리는 topic 형태의 goal

## 4. Published Topics
1. cmd_vel([geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)) : 로봇에 내리는 속도 명령

## 5. Services
1. ~make_plan(nav_msgs/GetPlan) : move_base에 주어진 pose를 위한 plan을 요청한다.
2. ~clear_unkown_space(std_srvs/Empty) : move_base에 알지 못하는 공간에 대한 costmap을 제거한다.
3. ~clear_costmaps(std_srvs/Empty) : costmap의 장애물을 제거한다.

## 6. Parameters
[move_base 위키 페이지](http://wiki.ros.org/move_base) 1.1.6 Parameters 참조