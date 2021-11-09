---
sort: 4
---

# Running Behaviors Without Operator

## 0. 참고 문헌

*- [FlexBe 홈페이지](http://philserver.bplaced.net/fbe/)*

*- [FlexBe Tutorials ROS 위키 페이지](http://wiki.ros.org/flexbe/Tutorials)*

*- [FlexBe Tutorials Running Behaviors Without Operator ROS 위키 페이지](http://wiki.ros.org/flexbe/Tutorials/Running%20Behaviors%20Without%20Operator)*

## 1. FlexBe만을 이용한 로봇 제어를 하는 경우
이 경우는 전체 시스템이 실행이 되었을 때, FlexBe를 최상위 제어기로 사용하는 경우이다.

1. onboard behavior engine luanch
```bash
roslaunch flexbe_onboard behavior_onboard.launch
```
2. ROS master가 연결된 상태에서 [Example Behavior] 이름의 behavior 실행
```bash
rosrun flexbe_widget be_launcher -b 'Example Behavior'
```

위 내용을 launch 파일 작성하여 launch하는 경우는 다음과 같이 작성한다.
```xml
<arg name="behavior_name" default="Example Behavior" />
<include file="$(find flexbe_onboard)/launch/behavior_onboard.launch" />
<node name="behavior_launcher" pkg="flexbe_widget" type="be_launcher" output="screen" args="-b '$(arg behavior_name)'" />
```

## 2. 상위 제어기에 FlexBe가 포함되어 실행되는 경우
이 경우에는 action call을 통해 behavior를 실행시킬 수 있는데, FlexBe 동작을 다른 상위 제어 인스턴스에 포함하는 경우에 사용된다.
1. onboard behavior engine launch
```bash
roslaunch flexbe_onboard behavior_onboard.launch
```
2. FlexBE action server 실행 : 이 action server는 action topic인 /flexbe/execute_behavior (flexbe_msgs/BehaviorExecution)를 받는다.([flexbe_behavior_engine 참고](https://github.com/team-vigir/flexbe_behavior_engine/blob/master/flexbe_msgs/action/BehaviorExecution.action))
```bash
rosrun flexbe_widget be_action_server
```

위 내용을 launch 파일에 포함시키고, 상위 제어기에서 behavior를 실행시키는 방법은 다음과 같다.
* luanch file:
```xml
<include file="$(find flexbe_onboard)/launch/behavior_onboard.launch" />
<node name="be_action_server" pkg="flexbe_widget" type="be_action_server" output="screen" respawn="true" />
```
* 상위 제어기에서의 behavior action 요청: 
```python
# python
# action client 생성
self._action_client = actionlib.SimpleActionClient('flexbe/execute_behavior', BehaviorExecutionAction)

# action goal 생성
self._action_goal = BehaviorExecutionGoal(behavior_name="Example Behavior")

# behavior 실행
self._action_client.send_goal(self._action_goal)
```

## 3. 사용자 인터페이스 이용
이 경우는 behavior 동작을 모니터링하거나, 명령 전송, 런타임 수정 등에 사용된다.
```bash
roslaunch flexbe_widget behavior_ocs.launch
```