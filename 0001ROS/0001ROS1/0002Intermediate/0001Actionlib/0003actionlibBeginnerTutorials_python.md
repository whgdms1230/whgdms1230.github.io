---
sort: 3
---

# actionlib beginner tutorials(python)

*- [actionlib tutorials 위키 페이지](http://wiki.ros.org/actionlib/Tutorials)*

## 1. Writing a Simple Action Server using the Execute Callback

### 1.1 Creating the Action Messages
이는 C++과 동일하므로 [actionlib beginner tutorials(c++)](/0001ROS/0001ROS1/0002Intermediate/0001Actionlib/0002actionlibBeginnerTutorials_cpp.html/)의 1.1을 참고한다.

### 1.2 Writing a Simple Server
#### 1.2.1 The Code
다음은 actionlib_tutorials/simple_action_servers/fibonacci_server.py 부분이다.
```python
#! /usr/bin/env python

import rospy

import actionlib

import actionlib_tutorials.msg

class FibonacciAction(object):
    # create messages that are used to publish feedback/result
    _feedback = actionlib_tutorials.msg.FibonacciFeedback()
    _result = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('fibonacci')
    server = FibonacciAction(rospy.get_name())
    rospy.spin()
```
#### 1.2.2 The Code, explained
* actionlib 라이브러리를 가져온다.
```python
import actionlib
```

* 앞서 만든 `.action`에 의해 빌드시 생성된 메세지들을 가져온다.
```python
import actionlib_tutorials.msg
```

* action server로 `SimpleActionServer`가 생성이 되고, `_action_name`은 action을 주고 받을 때 사용할 이름을 나타내고, `actionlib_tutorials.msg.FibonacciAction`과 같이 action type과 실행하기 위한 콜백함수(선택사항)을 인자로 선언한다. 이 콜백함수는 새로운 `goal`이 들어올 때마다 수행되도록 thread가 생성된다. 그리고 `auto_start`는 무엇이 수행되고 있는지 모르는 경우에는 항상 `False`로 두어야 한다.
```python
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
```

* `execute_cb` 콜백함수를 선언하며, `_feedback.sequence`를 초기화 한다.
```python
    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
```

* client로부터 요청받은 `goal`을 수행하며, 수행 도중 새로운 action이 요청되면 `is_preempt_requested()`가 호출되어 현재 `goal`에 대한 수행이 중단되고, 새로운 `goal`에 대한 작업을 수행한다.
```python
        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
```

* 피보나치 수열을 담은 `_feedback` 변수를 action server를 통해 publish하며, 이는 for문을 돌며 지속적으로 feedback 된다.
```python
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
```

* 모든 작업이 종료되고, `success`가 `true`상태라면 action server를 통해 `_result` 변수를 publish 한다.
```python
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
```

* main 함수는 노드 초기화 및 `FibonacciAction`을 실행한다.
```python
if __name__ == '__main__':
    rospy.init_node('fibonacci')
    server = FibonacciAction(rospy.get_name())
    rospy.spin()
```

## 2. Writing a Simple Action Client

### 2.1 The Code
다음은 actionlib_tutorials/simple_action_servers/fibonacci_client.py 부분이다.
```python
#! /usr/bin/env python

import rospy
from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
```

### 2.2 The Code, explained
* 앞서 만든 `.action`에 의해 빌드시 생성된 메세지들을 가져온다.
```python
import actionlib_tutorials.msg
```

* SimpleActionClient는 client와 server가 주고받을 action 이름을 정의하고, 메세지 타입을 정의한다.
```python
    client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)
```

* action server가 수행될 때까지 기다리도록 `wait_for_server()` 함수를 수행한다.
```python
    client.wait_for_server()
```

* `goal`을 설정하고 `send_goal()` 함수를 통해 goal을 요청한다.
```python
    # Creates a goal to send to the action server.
    goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

    # Sends the goal to the action server.
    client.send_goal(goal)
```

* `wait_for_result()`함수는 action server로 goal을 요청하고 결과를 콜백 받을 때까지 기다리며, 서버가 결과를 보내오면 client를 종료한다.
```python
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult
```
