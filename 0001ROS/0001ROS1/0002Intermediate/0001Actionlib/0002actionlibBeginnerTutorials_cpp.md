---
sort: 2
---

# actionlib beginner tutorials(c++)

*- [actionlib tutorials 위키 페이지](http://wiki.ros.org/actionlib/Tutorials)*

## 1. Writing a Simple Action Server using the Execute Callback

### 1.1 Creating the Action Messages
`.action`은 `goal`, `result`, `feedback`을 정의한다.
다음은 [Fibonacci.action]() 파일에 정의된 내용이다.
```text
#goal definition
int32 order
---
#result definition
int32[] sequence
---
#feedback
int32[] sequence
```

다음으로, action 메세지를 생성하기 위해 CMakeLsts.txt에 해당 내용을 추가해야한다.
```cmake
find_package(catkin REQUIRED COMPONENTS actionlib_msgs)

add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)
```

또한, package.xml에도 다음을 추가해야 한다.
```xml
<exec_depend>message_generation</exec_depend>
```

### 1.2 Writing a Simple Server
#### 1.2.1 The Code
다음은 actionlib_tutorials/src/fibonacci_server.cpp 부분이다.

```c++
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>

class FibonacciAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;

public:

  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~FibonacciAction(void)
  {
  }

  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}
```

#### 1.2.2 The Code Explained

* actionlib/server/simple_action_server.h는 simple action을 구현하는 데 사용된다.
```cpp
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
```

* 위에서 만든 `.action`에 의해 빌드시 생성된 FibonacciAction.h를 가져온다.
```cpp
#include <actionlib_tutorials/FibonacciAction.h>
```

* 다음은 `FibonacciAction` 클래스의 `protected` 변수로, `NodeHandle`은 action server를 핸들링하는데 사용되며, `SimpleActionServer`는 action server를 구현하는데 사용된다. `action_name_`은 action을 주고 받을 때 사용할 이름을 나타내고, `feedback_`과 `result_`는 action 수행의 결과를 담기 위한 변수이다.
```cpp
class FibonacciAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;
```

* 클래스 생성자를 통해 action server `as_`가 생성되고, 이는 NodeHandle, action 이름, 실행하기 위한 콜백함수(선택사항)을 인자로 받는다.
```cpp
public:

  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~FibonacciAction(void)
  {
  }
```

* 실행 콜백 함수 `executeCB`는 goal 메세지를 포인터 형태로 받는다.(여기서 자료형은 메세지 유형의 뒤에 `ConstPtr`을 추가하여 생성된 boost shared pointer이다.)
```cpp
  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
  {
```

* `feedback_.sequence`를 초기화한다.
```cpp
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
```

* client로부터 요청받은 `goal`을 수행한다. 수행 도중 사용자가 새로운 action을 요청하여 `isPreemptRequested()`가 호출되면 현재 `goal`에 대한 수행이 중단되고, 새로운 `goal`에 대한 작업을 수행한다.
```cpp
    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
```

* 피보나치 수열을 담은 `feedback_` 변수를 action server를 통해 publish하며, 이는 for문을 돌며 지속적으로 feedback 된다.
```cpp
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }
```

* 모든 작업이 종료되고, `success`가 `true`상태라면 action server를 통해 `result_` 변수를 publish 한다.
```cpp
    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
```

* main 함수는 노드 초기화 및 `FibonacciAction`을 실행한다.
```cpp
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}
```

### 1.3 Compiling
생성한 action server 소스 코드를 Compile 하기 위해 CmakeLists.txt에 다음을 추가적으로 작성해야 한다.
```cmake
add_executable(fibonacci_server src/fibonacci_server.cpp)

target_link_libraries(
  fibonacci_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_server
  ${actionlib_tutorials_EXPORTED_TARGETS}
)
```

## 2. Writing a Simple Action Client

### 2.1 The Code
다음은 actionlib_tutorials/src/fibonacci_client.cpp 부분이다.
```cpp
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
```

### 2.2 The Code Explained
* actionlib/client/simple_action_client.h는 simple action client을 구현하는 데 사용된다. actionlib/client/terminal_state.h는 가능한 목표 상태를 정의한다.
```cpp
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
```

* 위에서 만든 `.action`에 의해 빌드시 생성된 FibonacciAction.h를 가져온다.
```cpp
#include <actionlib_tutorials/FibonacciAction.h>
```

* SimpleActionClient는 메세지 타입을 정의하고, action 이름과 actionlib 스레드를 spin 할 boolean 변수를 인자로 입력한다.
```cpp
int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);
```

* action server가 수행될 때까지 기다리도록 `waitForServer()`함수를 수행한다.
```cpp
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
```

* `goal`을 설정하고 `sendGoal()`함수를 통해 goal을 요청한다.
```cpp
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);
```

* `waitForResult()`함수는 action server로 goal을 요청한 이후 결과를 콜백받을 때까지 대기한다. 여기서 대기 제한 시간을 30초로 설정하였으며, 만약 30초 이내에 callback을 받을 경우 `true`를 반환한다.
```cpp
  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
```

* 만약 제한 시간 이내에 콜백을 받았다면, `getState()`함수로 goal 요청에 대한 상태정보를 받아올 수 있다.
```cpp
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
```

### 2.3 Compling
생성한 action client 소스 코드를 Compile 하기 위해 CmakeLists.txt에 다음을 추가적으로 작성해야 한다.
```cmake
add_executable(fibonacci_client src/fibonacci_client.cpp)

target_link_libraries( 
  fibonacci_client
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_client
  ${actionlib_tutorials_EXPORTED_TARGETS}
)
```