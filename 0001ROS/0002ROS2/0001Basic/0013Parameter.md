---
sort: 13
---

# Parameter

## 0. 참고 문헌
*- ROS 2로 시작하는 로봇 프로그래밍(표윤석, 임태훈)*

*- [Using parameters in a class (C++)](https://docs.ros.org/en/foxy/Tutorials/Using-Parameters-In-A-Class-CPP.html)*

*- [Using parameters in a class (Python)](https://docs.ros.org/en/foxy/Tutorials/Using-Parameters-In-A-Class-Python.html)*

## 1. Parameter

[파라미터](https://docs.ros.org/en/foxy/Tutorials/Parameters/Understanding-ROS2-Parameters.html)는 각 노드의 `parameter server`를 통해 외부 `parameter client`와 통신하여 파라미터를 변경하는 것이다. 이는 노드 내부 또는 외부에서 노드 내 매개변수를 `Set` 하거나 `Get` 할 수 있도록 한다.

모든 노드는 노드 자신만의 `parameter server`를 가지고 있고, 또한 각 노드는 `parameter client`를 가질수도 있어서 자기 자신의 파라미터 및 다른 노드의 파라미터를 읽고 쓸 수 있다. 이는 각 노드의 다양한 매개변수를 글로벌 매개변수처럼 사용할 수 있게 하며, 동적으로 변화 가능한 프로세스를 만들 수 있다.

## 2. C++

다음은 parameter를 사용한 노드의 간단한 [예제](https://docs.ros.org/en/foxy/Tutorials/Using-Parameters-In-A-Class-CPP.html#write-the-c-node)이다.

```cpp
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;

class ParametersClass: public rclcpp::Node
{
  public:
    ParametersClass()
      : Node("parameter_node")
    {
      this->declare_parameter<std::string>("my_parameter", "world");
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&ParametersClass::respond, this));
    }
    void respond()
    {
      this->get_parameter("my_parameter", parameter_string_);
      RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
    }
  private:
    std::string parameter_string_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametersClass>());
  rclcpp::shutdown();
  return 0;
}
```

* 먼저 `rclcpp::Node`를 상속받는 `ParametersClass` 노드를 생성하고 초기화 한다. 생성자에서 `declare_parameter`를 이용하여 `my_parameter`라는 파라미터를 생성하고, default 값으로 `world`를 사용한다. 다음으로, `timer_`를 설정하여, `respond`라는 함수를 주기적으로 콜백한다.

```cpp
class ParametersClass: public rclcpp::Node
{
  public:
    ParametersClass()
      : Node("parameter_node")
    {
      this->declare_parameter<std::string>("my_parameter", "world");
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&ParametersClass::respond, this));
    }
```

* `respond` 함수는 `get_parameter`를 이용하여 앞서 생성한 `my_parameter`의 값을 `parameter_string_`이라는 변수에 저장하고, 이를 로그로 남기는 역할을 한다.

```cpp
void respond()
{
  this->get_parameter("my_parameter", parameter_string_);
  RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
}
```

* `ParametersClass` 클래스는 다음과 같은 변수를 갖늗다.

```cpp
private:
  std::string parameter_string_;
  rclcpp::TimerBase::SharedPtr timer_;
```

* 메인 함수에서는 `ParametersClass` 노드를 실행시킨다.

```cpp
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametersClass>());
  rclcpp::shutdown();
  return 0;
}
```

## 3. Python

다음은 parameter를 사용한 노드의 간단한 [예제](https://docs.ros.org/en/foxy/Tutorials/Using-Parameters-In-A-Class-Python.html#write-the-python-node)이다.

```python
import rclpy
import rclpy.node

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('my_parameter', 'world')

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

* 먼저 의존성이 있는 `rclpy` 및 `rclpy.node`를 import한다.

```python
import rclpy
import rclpy.node
```

* 다음으로 `rclpy.node.Node`를 상속받는 `MinimalParam` 노드를 생성한다. 노드를 초기화하고, `timer_callback`이라는 함수를 주기적으로 호출하도록 설정한다. `declare_parameter`를 이용해 파라미터를 선언하는데, 파라미터의 이름은 `my_parameter`이고, default 값은 `world`이다.

```python
class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('my_parameter', 'world')
```

* `timer_callback` 함수는 `get_parameter`를 통해 앞에서 생성한 `my_parameter`의 값을 가져와 `my_param`에 저장하고, 이를 로그에 남기는 역할을 한다. 여기서 `set_parameters`는 `my_parameter`의 값을 다시 `world`라는 default 값으로 저장을 하며, 이는 외부에서 변경이 되어도 일시적으로만 적용하고, 계속해서 default 값으로 돌아가게 하는 역할을 한다.

```python
def timer_callback(self):
    my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

    self.get_logger().info('Hello %s!' % my_param)

    my_new_param = rclpy.parameter.Parameter(
        'my_parameter',
        rclpy.Parameter.Type.STRING,
        'world'
    )
    all_new_parameters = [my_new_param]
    self.set_parameters(all_new_parameters)
```

* 메인 함수에서는 `MinimalParam` 노드를 실행시킨다.

```python
def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## 4. parameter 변경

### 4.1 Command Line
먼저 콘솔창에서 command line으로 parameter를 변경하는 방법이다.
```bash
ros2 param set <node_name> my_parameter earth
```

### 4.2 Launch file
다음으로 런치 파일에서 실행 시 파라미터를 초기화하는 방법이다.
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="parameter_node",
            name="custom_parameter_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
```

### 4.3 YAML file
다음은 위와 동일하게 런치 파일에서 실행 시 파라미터를 초기화 하는데, yaml 파일에 정의되어 있는 값을 가져와서 초기화하는 방법이다.
```python
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cpp_parameters'),
        'config',
        'parameters.yaml'
        )

    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="parameter_node",
            name="custom_parameter_node",
            output="screen",
            emulate_tty=True,
            parameters=[config]
      )
   ])
```

다음은 사용된 yaml 파일 내용이다. `ros__parameters` 이전 태그에는 `namespace`와 `node name`에 해당하는 태그를 작성하고, `ros__parameters` 태그는 해당 태그를 작성해야만 그 아래 설정된 값들이 ROS 파라미터로 등록이 되며, 등록 타입은 bool, int, double, string, array 등이다.
```yaml
/custom_parameter_node:
    ros__parameters:
        my_parameter: "earth"
```