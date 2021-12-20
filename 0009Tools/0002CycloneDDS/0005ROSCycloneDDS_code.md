---
sort: 5
---

# ROS-CycloneDDS Code

## 0. 참고 문헌
*- [OSRF RMF free fleet github pages](https://github.com/open-rmf/free_fleet)*

*- [Cyclone Bridge](https://github.com/whgdms1230/cyclone_bridge)*

*- [Cyclone Ros1 Node](https://github.com/whgdms1230/cyclone_ros1_node)*

*- [Cyclone Ros Node](https://github.com/whgdms1230/cyclone_ros2_node)*


## 1. cyclone_bridge
cyclone_bridge의 구조는 다음과 같다.

```bash
cyclone_bridge
├── cmake
│   └── cyclone_bridge-config.cmake.in
├── CMakeLists.txt
├── include
│   └── cyclone_bridge
│       ├── messages
│       │   ├── IntNumber.hpp
│       │   ├── Msg.hpp
│       │   └── StrString.hpp
│       ├── ROS1Bridge.hpp
│       ├── ROS1Config.hpp
│       ├── ROS2Bridge.hpp
│       └── ROS2Config.hpp
├── package.xml
├── README.md
└── src
    ├── dds_utils
    │   ├── common.cpp
    │   ├── common.hpp
    │   ├── DDSPublishHandler.hpp
    │   └── DDSSubscribeHandler.hpp
    ├── messages
    │   ├── Messages.c
    │   ├── Messages.h
    │   └── Messages.idl
    ├── ROS1Bridge.cpp
    ├── ROS1Impl.cpp
    ├── ROS1Impl.hpp
    ├── ROS2Bridge.cpp
    ├── ROS2Impl.cpp
    └── ROS2Impl.hpp
```

* `cyclone_birdge-config.cmake.in`은 cmake 파일을 정의하기 위해 사용된 파일이다.
* `include/cyclone_bridge`에는 DDS통신으로 받아온 데이터를 ROS 패키지 안에서 저장하기 위해 `message`폴더 안에 헤더파일로 정의되어 있다. 그리고, `*Bridge.hpp` 파일은 ros 노드에서 `cyclone_bridge`의 함수를 호출하기 위해 사용되며 `*Config.hpp`파일은 dds 통신시 필요한 configuration을 정의하기 위해 사용된다.
* `src/dds_tuils`는 dds 통신에 필요한 util 파일들이며, `common.cpp`, `common.hpp`는 ros의 string 자료형 변수를 dds의 char* 자료형 변수에 할당하기 위한 `dds_string_alloc_and_copy()`함수가 정의되어 있으며, `DDSPublishHandler.hpp`와 `DDSSubscribeHandler.hpp`에는 DDS 통신에 사용되는 handler를 정의한 클래스가 구현되어 있다.
* `src` 내부의 `*Bridge.cpp`는 ros 노드에서 `cyclone_bridge` 함수를 호출하기 위한 함수를 정의하였으며, 해당 함수를 이용하여 dds 통신 구현체인 `impl`의 함수를 사용하도록 구현되어 있다. `*Impl.cpp`, `*Impl.hpp`에는 실제 dds 통신을 구현하는 구현체가 들어있으며, 해당 함수들을 이용하여 dds 메시지 통신을 하도록 구현되어있다.

### 1.1 Bridge
ROS1Bridge.cpp를 보면 다음과 같다.

* 먼저 ROS1Bridge 객체를 만들 때, ROS1Config의 configuration을 가져온다. 다음으로 `participant`를 만들고, `DDSPublisherHandler`와 `DDSSubscribeHandler`객체를 이용하여 dds 통신할 객체를 생성한다.
```cpp
ROS1Bridge::SharedPtr ROS1Bridge::make(const ROS1Config& _config)
{
  SharedPtr ros1_bridge = SharedPtr(new ROS1Bridge(_config));

  dds_entity_t participant = dds_create_participant(
      static_cast<dds_domainid_t>(_config.dds_domain), NULL, NULL);
  if (participant < 0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  dds::DDSPublishHandler<CycloneBridgeData_Msg>::SharedPtr send_pub(
      new dds::DDSPublishHandler<CycloneBridgeData_Msg>(
          participant, &CycloneBridgeData_Msg_desc,
          _config.dds_ros1_to_ros2_topic));

  dds::DDSSubscribeHandler<CycloneBridgeData_Msg>::SharedPtr 
      read_sub(
          new dds::DDSSubscribeHandler<CycloneBridgeData_Msg>(
              participant, &CycloneBridgeData_Msg_desc,
              _config.dds_ros2_to_ros1_topic));

  if (!send_pub->is_ready() || !read_sub->is_ready())
    return nullptr;

  ros1_bridge->impl->start(ROS1Impl::Fields{
      std::move(participant),
      std::move(send_pub),
      std::move(read_sub)});
  return ros1_bridge;
}
```

* ros1 노드에서 호출할 `send()`함수와 `read()`함수로, 해당 함수들이 호출되면 실제 dds 통신을 구현하는 impl의 `send()`함수와 `read()`함수를 호출한다.
```cpp
bool ROS1Bridge::send(const messages::Msg& ros1_to_ros2_msg)
{
  return impl->send(ros1_to_ros2_msg);
}

bool ROS1Bridge::read(messages::Msg& ros2_to_ros1_msg)
{
  return impl->read(ros2_to_ros1_msg);
}
```

### 1.2 Impl
dds 통신을 구현하는 ROS1Impl.cpp를 보면 다음과 같다.

* 먼저 `send()`함수로, cyclone_bridge의 messages 폴더 내에 선언된 메시지 형태의 자료구조(여기서는 `Msg`)를 받아 Messages.h에 정의된 메시지 형태로 변환하여 dds 함수를 이용하여 전송하는 함수이다. 
* 먼저 `CycloneBridgeData_Msg__alloc()`을 이용하여 msg를 동적 할당 하고, msg에 데이터를 저장한다. 이 때, string의 경우 `common`에 정의되어 있는  `dds_string_alloc_and_copy()`함수를 이용하여 char* 자료형으로 변환하여 대입한다.
* 다음으로 `DDSPublishHandler`에 정의되어 있는 `write()`함수를 이용하여 msg를 전송하고, `CycloneBridgeData_Msg_free()`함수로 동적 할당을 해제한다.
```cpp
bool ROS1Bridge::ROS1Impl::send(const messages::Msg& ros1_to_ros2_msg)
{
  CycloneBridgeData_Msg* msg = CycloneBridgeData_Msg__alloc();
  msg->cnt.int_num = ros1_to_ros2_msg.cnt.int_num;
  msg->messages.messages = common::dds_string_alloc_and_copy(ros1_to_ros2_msg.messages.messages);

  bool sent = fields.send_pub->write(msg);
  CycloneBridgeData_Msg_free(msg, DDS_FREE_ALL);
  return sent;
}
```

* 다음으로 `read()`함수로, Messages.h에 정의된 메시지 형태의 데이터를 `DDSSubscribeHandler`에 정의된 `read()`를 이용하여 msg를 읽고, 읽어온 msg 데이터를 변환하여 cyclone_bridge의 messages 폴더 내에 선언된 메시지 형태의 자료구조(여기서는 `Msg`)에 저장하는 함수이다.
```cpp
bool ROS1Bridge::ROS1Impl::read(messages::Msg& ros2_to_ros1_msg)
{
  auto msg = fields.read_sub->read();
  if (!msg.empty())
  {
    ros2_to_ros1_msg.cnt.int_num = msg[0]->cnt.int_num;
    ros2_to_ros1_msg.messages.messages = std::string(msg[0]->messages.messages);

    return true;
  }
  return false;
}
```

## 2. cyclone_ros1_node
cyclone_ros1_node의 구조는 다음과 같다.

```bash
cyclone_ros1_node
├── CMakeLists.txt
├── launch
│   └── cyclone_ros1_node.launch
├── msg
│   ├── IntNumber.msg
│   ├── Msg.msg
│   └── StrString.msg
├── package.xml
├── README.md
└── src
    ├── main.cpp
    ├── ROS1NodeConfig.cpp
    ├── ROS1NodeConfig.hpp
    ├── ROS1Node.cpp
    └── ROS1Node.hpp
```

* `msg`는 토픽의 pub 또는 sub을 하기 위해 message generation한 폴더이다.
* `main.cpp`는 메인 함수가 구현되어 있는 파일이다.
* `ROS1NodeConfig`는 노드에 필요한 configuration을 정의한 파일이다.
* `ROS1Node`는 ros1 노드로써, 토픽을 pub/sub 하고, dds 통신을 위한 handler가 선언되어 있는 파일이다.

`ROS1Node.cpp`의 일부를 설명하면 다음과 같다.

* 먼저 ROS1Node를 생성하는 부분이다. cyclone_bridge에서 config를 가져오고, 해당 config를 이용하여 cyclone_bridge의 ROS1Bridge 객체를 생성한다. 이렇게 생성된 ros1_node는 ros1_bridge 객체를 이용하여 dds 통신을 할 것이다.
```cpp
ROS1Node::SharedPtr ROS1Node::make(const ROS1NodeConfig& _config)
{
  SharedPtr ros1_node = SharedPtr(new ROS1Node(_config));
  ros1_node->node.reset(new ros::NodeHandle("ros1_node"));

  /// Starting the ros1 node
  ROS1Config ros1_config = _config.get_ros1_config();
  ROS1Bridge::SharedPtr ros1_bridge = ROS1Bridge::make(ros1_config);
  if (!ros1_bridge)
    return nullptr;

  ros1_node->start(Fields{
      std::move(ros1_bridge)
  });

  return ros1_node;
}
```

* start 함수는 메시지를 입력받을 `send_topic_sub`과 리턴받은 값을 출력할 `read_topic_pub` 토픽을 생성한다. 그리고, ros2 노드로부터 리턴값을 받기 위해 `read_thread`라는 쓰레드를 생성한다.
```cpp
void ROS1Node::start(Fields _fields)
{
  fields = std::move(_fields);

  read_rate.reset(new ros::Rate(ros1_node_config.read_frequency));

  send_topic_sub = node->subscribe(
      ros1_node_config.ros1_to_ros2_topic, 1, &ROS1Node::send_topic_cb, this);

  read_topic_pub = node->advertise<cyclone_ros1_node::Msg>(ros1_node_config.ros2_to_ros1_topic, 10);

  read_thread = std::thread(std::bind(&ROS1Node::read_thread_fn, this));
}
```

* dds 토픽을 send하는 부분이다. `/ros1_to_ros2_topic` 메세지를 subscribe하면 `send_topic_cb()`함수가 callback되는데, 해당 메시지에서 넘어온 int_num과, message 정보를 이용하여 업데이트 한다.
* 업데이트 이후 `send()`함수를 호출하는데, 여기서 cyclone_birdge에 정의된 messages::Msg형 변수에 데이터를 저장하고, bridge의 send함수로 전송한다. 
```cpp
void ROS1Node::send_topic_cb(
    const cyclone_ros1_node::Msg& _msg)
{
  new_number = _msg.cnt.int_num;
  new_string = _msg.messages.messages;
  send();
}

void ROS1Node::send()
{
  messages::Msg ros1_to_ros2_msg;
  ros1_to_ros2_msg.cnt.int_num = new_number;
  ros1_to_ros2_msg.messages.messages = new_string;

  fields.ros1_bridge->send(ros1_to_ros2_msg);
}
```

* 다음은 ros2 노드로부터 메시지를 read하기 위한 부분으로, `read_thread_fn()` 쓰레드에서 `read()`함수를 주기적으로 호출하고, bridge의 read함수로 해당 토픽을 읽어오게 되면 `ros2_to_ros1_msg`에 담겨져 리턴된 값을 `read_topic_pub`의 메시지 구조에 담아 publish하게 된다.
```cpp
void ROS1Node::read()
{
  messages::Msg ros2_to_ros1_msg;
  if (fields.ros1_bridge->read(ros2_to_ros1_msg))
  {
    cyclone_ros1_node::Msg new_msg;

    new_msg.cnt.int_num = ros2_to_ros1_msg.cnt.int_num;
    new_msg.messages.messages = ros2_to_ros1_msg.messages.messages;

    read_topic_pub.publish(new_msg);
  }
}

void ROS1Node::read_thread_fn()
{
  while (node->ok())
  {
    read_rate->sleep();
    
    // read message from DDS
    read();
  }
}
```

## 5. cyclone_ros2_node
cyclone_ros1_node의 구조는 다음과 같다.

```bash
cyclone_ros2_node
├── cyclone_ros2_msgs
│   ├── CMakeLists.txt
│   ├── msg
│   │   ├── IntNumber.msg
│   │   ├── Msg.msg
│   │   └── StrString.msg
│   ├── package.xml
│   └── README.md
└── cyclone_ros2_node
    ├── CMakeLists.txt
    ├── launch
    │   └── cyclone_ros2_node.xml
    ├── package.xml
    ├── README.md
    └── src
        ├── main.cpp
        ├── ROS2NodeConfig.cpp
        ├── ROS2NodeConfig.hpp
        ├── ROS2Node.cpp
        └── ROS2Node.hpp

```

* `cyclone_ros2_msgs` 패키지는 토픽의 pub 또는 sub을 하기 위해 message generation하기 위한 패키지이다.
* `main.cpp`는 메인 함수가 구현되어 있는 파일이다.
* `ROS2NodeConfig`는 노드에 필요한 configuration을 정의한 파일이다.
* `ROS2Node`는 ros2 노드로써, 토픽을 pub/sub 하고, dds 통신을 위한 handler가 선언되어 있는 파일이다.

`ROS2Node.cpp`의 일부를 설명하면 다음과 같다.

* 먼저 ROS2Node를 생성하는 부분이다. cyclone_bridge에서 config를 가져오고, 해당 config를 이용하여 cyclone_bridge의 ROS2Bridge 객체를 생성한다. 이렇게 생성된 ros2_node는 ros2_bridge 객체를 이용하여 dds 통신을 할 것이다.
```cpp
ROS2Node::SharedPtr ROS2Node::make(
    const ROS2NodeConfig& _config, const rclcpp::NodeOptions& _node_options)
{
  // Starting the free fleet server node
  SharedPtr ros2_node(new ROS2Node(_config, _node_options));

  ROS2Config ros2_config =
      ros2_node->ros2_node_config.get_ros2_config();
  ROS2Bridge::SharedPtr ros2_bridge = ROS2Bridge::make(ros2_config);
  if (!ros2_bridge)
    return nullptr;

  ros2_node->start(Fields{
    std::move(ros2_bridge)
  });

  return ros2_node;
}
```

* start 함수는 dds로 읽어온 함수를 주기적으로 콜백할 객체를 만들고, read한 메시지를 publish하기 위한 토픽을 생성한다.
```cpp
void ROS2Node::start(Fields _fields)
{
  fields = std::move(_fields);

  read_callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  read_timer = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&ROS2Node::read, this),
      read_callback_group);

  ros1_to_ros2_msg_pub =
      create_publisher<cyclone_ros2_msgs::msg::Msg>(
          ros2_node_config.ros1_to_ros2_topic, 10);
}
```

* 다음은 주기적으로 호출되는 `read()` 함수로, ros2_bridge의 read 함수를 이용하여 메시지가 들어오면 `ros1_to_ros2_msg`에 저장되어 있는 메시지를 publish할 메시지에 저장하여 publish 한다.
```cpp
void ROS2Node::read()
{
  messages::Msg ros1_to_ros2_msg;
  if (fields.ros2_bridge->read(ros1_to_ros2_msg))
  {
    cyclone_ros2_msgs::msg::Msg new_msg;
    new_msg.cnt.int_num = ros1_to_ros2_msg.cnt.int_num;
    new_msg.messages.messages = ros1_to_ros2_msg.messages.messages;

    return_number = new_msg.cnt.int_num;
    return_string = new_msg.messages.messages;

    ros1_to_ros2_msg_pub->publish(new_msg);

    send();
  }
}
```

* 다음은 `send()`함수로 `read()`함수에서 읽어온 메시지를 저장한 변수를 다시 담아서 리턴하는 함수이다. 이 때도 ros2_bridge의 send 함수를 이용하여 메시지를 전송한다.
```cpp
void ROS2Node::send()
{
  messages::Msg ros2_to_ros1_msg;
  ros2_to_ros1_msg.cnt.int_num = return_number;
  ros2_to_ros1_msg.messages.messages = return_string;

  fields.ros2_bridge->send(ros2_to_ros1_msg);
}
```

## 6. Customizing 방법

여기서는 DDS 통신을 이용하여 메시지를 추가하거나 메시지 구조를 변경하는 방법 등에 대해 설명한다. 그 이외의 dds 통신을 기반으로 node의 응용은 사용자의 능력에 따라 달라질 것 같다.

### 6.1 cyclone_bridge
1. 새로운 메시지 구조를 만들기 위해 Messages.idl에 원하는 메시지 구조를 입력한다.
2. idlc를 이용하여 Messages.c와 Message.h를 생성한다.
3. Messages.idl에 선언한 메시지를 담을 수 있는 메시지를 `cyclone_bridge/include/cyclone_bridge/messages`에 정의한다.
4. DDS로 통신하고자 하는 메시지 형태를 정한다.
5. 통신하고자 하는 메시지들의 토픽명을 config에 저장한다.
6. 통신하고자 하는 메시지 형태마다 Handler를 이용하여 send용 구현체와 read용 구현체를 만든다.
7. bridge에 해당하는 자료구조를 인자로 받은 send용 함수와 read용 함수를 만든다.

### 6.2 ros node
1. 새로운 메시지 형태를 ros 메시지로 사용하고자 한다면 동일한 메시지 구조로 message generation을 한다.(선택 사항)
2. ros node에서는 cyclone_bridge의 config와 bridge 객체를 가져오므로, cyclone_bridge에서 message 형태를 가져와 해당하는 메시지 구조에 담아서 read 또는 send하면 dds 통신을 활용할 수 있다.