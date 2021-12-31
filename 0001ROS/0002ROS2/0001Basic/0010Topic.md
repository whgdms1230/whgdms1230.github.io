---
sort: 10
---

# Topic

## 0. 참고 문헌
*- ROS 2로 시작하는 로봇 프로그래밍(표윤석, 임태훈)*

*- [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)*

*- [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)*


## 1. Topic
토픽은 비동기식 단방향 메시지 송수신 방식으로 Publisher와 Subscriber 간의 통신으로 이루어진다. Topic은 1:1, 1:N, N:1, N:N 통신 모두 가능하다.

## 2. C++

### 2.1 Publisher
다음은 topic publisher의 간단한 [예제](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#write-the-publisher-node)이다.

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

* 먼저 C++ publisher 노드에서 사용할 헤더를 선언한다. 특히 `rclcpp/rclcpp.hpp`는 ROS2에서 C++을 사용하기 위한 기본적인 기능을 포함하고 있으며, `std_msgs/msg/string.hpp`는 string으로 publish하기 위한 메시지 타입을 선언한 것이다.

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
```

* 다음으로 `MinimalPublisher`라는 노드 클래스를 생성하며, `rclcpp::Node`를 상속받는다.

```cpp
class MinimalPublisher : public rclcpp::Node
```

* `MinimalPublisher`의 생성자로, 노드를 초기화하고, `std_msgs::msg::String` 타입의 `topic`의 이름을 갖는 publisher를 생성한다. 또한 주기적으로 토픽을 publish하기 위해 `create_wall_timer()`를 이용하여 `timer_`를 초기화하고, 500ms마다 `timer_callback` 함수를 호출한다.

```cpp
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }
```

* `timer_callback()`함수를 정의한다. `std_msgs::msg::String()` 타입의 message 변수를 선언하여. 데이터를 입력하고 publisher를 이용하여 message를 publish한다.

```cpp
private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
```

* `timer_`와 `publisher_`, `count_` 변수를 선언한다.

```cpp
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
size_t count_;
```

* 마지막으로 `main()`함수에서 `MinimalPublisher` 객체를 가져와 실행한다.

```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

### 2.2 Subscriber
다음은 topic subscriber의 간단한 [예제](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#write-the-subscriber-node)이다.

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

* publisher 노드와 동일하게 subscriber 노드에서 사용할 헤더를 선언한다. 특히 `rclcpp/rclcpp.hpp`는 ROS2에서 C++을 사용하기 위한 기본적인 기능을 포함하고 있으며, `std_msgs/msg/string.hpp`는 string으로 publish하기 위한 메시지 타입을 선언한 것이다.

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
```

* 다음으로 `MinimalPublisher`라는 노드 클래스를 생성하며, `rclcpp::Node`를 상속받는다.

```cpp
class MinimalSubscriber : public rclcpp::Node
```

* `MinimalSubscriber`의 생성자로, 노드를 초기화하고, `std_msgs::msg::String` 타입의 `topic`이라는 이름을 갖는 subscriber를 생성한다. 해당 `topic`으로 메시지를 subscirbe하면, `topic_callback`함수를 호출한다.

```cpp
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }
```

* `topic_callback()` 함수는 해당 토픽으로 받은 message를 RCLCPP_INFO로 로그를 남긴다.

```cpp
private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
```

* `main()` 함수에서 `MinimalSubscriber` 객체를 불러와 subscriber 노드를 실행시킨다.

```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

## 3. Python

### 3.1 Publisher
다음은 topic publisher의 간단한 [예제](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-publisher-node)이다.

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

* 먼저 ros2 python 노드에 사용되는 `rclpy`와 `rclpy.node`의 `Node`를 import한다.

```python
import rclpy
from rclpy.node import Node
```

* topic의 메시지 타입으로 사용할 `String`을 import 한다.

```python
from std_msgs.msg import String
```

* `Node`를 상속받는 `MinimalPublisher` 클래스를 생성한다.

```python
class MinimalPublisher(Node):
```

* 생성자에서 node를 초기화하고, `topic`이라는 이름을 갖는 publisher를 `create_publisher` 함수를 이용하여 생성한다. 또한 0.5초마다 주기적으로 publish하도록 `timer`를 정의하고, 해당 `timer`마다 `timer_callback()`함수를 호출한다.

```python
def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
```

* `timer_callback()` 함수는 `String` 타입의 message를 만들어 publish한다.

```python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```

* 마지막으로 `main()`함수에서 MinimalPublisher 객체를 생성함으로써, publisher 노드를 시작한다.

```python
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

### 3.2 Subscriber
다음은 topic subscriber의 간단한 [예제](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-subscriber-node)이다.

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

* publisher와 동일하게 ros2 python 노드에 필요한 클래스를 import하고, subscribe할 메시지 타입인 `String`을 import한다.

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
```

* Subscribe 노드인 `MinimalSubscriber` 클래스를 생성한다.

```python
class MinimalSubscriber(Node):
```

* 생성자에서 노드를 초기화하고, `create_subscription()`을 이용하여 `topic`이라는 이름을 갖는 subscriber 객체를 생성한다. 해당 subscriber 객체는 토픽을 subscribe하면 `listener_callback` 함수를 호출한다.

```python
def __init__(self):
    super().__init__('minimal_subscriber')
    self.subscription = self.create_subscription(
        String,
        'topic',
        self.listener_callback,
        10)
    self.subscription  # prevent unused variable warning
```

* subscribe시 호출되는 `listener_callback()`함수를 정의하며, 호출 시 subscribe한 메시지의 로그를 남긴다.

```python
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

* 마지막으로 `main()`함수에서 `MinimalSubscriber` 객체를 생성함으로써, subscribe 노드를 실행시킨다.

```python
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
```