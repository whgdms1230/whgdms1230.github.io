---
sort: 4
---

# ROS-CycloneDDS Introduce

## 0. 참고 문헌
*- [OSRF RMF free fleet github pages](https://github.com/open-rmf/free_fleet)*

*- [Cyclone Bridge](https://github.com/whgdms1230/cyclone_bridge)*

*- [Cyclone Ros1 Node](https://github.com/whgdms1230/cyclone_ros1_node)*

*- [Cyclone Ros Node](https://github.com/whgdms1230/cyclone_ros2_node)*


## 1. 프로젝트 개요
이 프로젝트는 ROS 1.0과 ROS 2.0과의 통신 구현체를 구현한 cyclone_bridge 패키지를 이용하여 ROS 1.0 노드(cyclone_ros1_node)와 ROS 2.0 노드(cyclone_ros2_node) 간의 메시지 통신을 하는 간단한 예제이다. 해당 프로젝트는 OSRF에서 개발한 [RMF](https://osrf.github.io/ros2multirobotbook/intro.html) 시스템 중 [Free Fleet](https://github.com/open-rmf/free_fleet)의 구현체를 베이스로 구현하였다.

동작 시퀀스는 다음과 같다.

* cyclone_ros1_node는 `/ros1_to_ros2_topic` 토픽을 subscribe하여 send할 메시지를 입력받고, cyclone_bridge의 send 함수를 이용하여 해당 메시지를 전송한다.
* cyclone_ros2_node는 cyclone_bridge의 read 함수를 이용하여 해당 토픽의 dds 메시지를 읽어오면, 해당 메시지를 `/ros1_to_ros2_topic` 토픽 메시지에 저장하여 publish하고, 다시 send 함수를 이용하여 해당 메시지를 cyclone_ros1_node에 전송한다.
* cyclone_ros1_node에서 리턴된 값을 읽게 되면 `/ros2_to_ros2_topic` 토픽 메시지에 저장하여 publish 함으로써, `/ros1_to_ros2_topic`으로 publish한 메시지가 ros2 노드로 전송되고, 리턴값이 ros1 노드로 다시 돌아온 것을 확인할 수 있다.

## 2. 설치 및 실행

### 2.1 cyclone_ros1_node

cyclone_ros1_node 패키지의 설치 방법은 다음과 같다.

```bash
mkdir -p cyclone_ros1_ws/src
cd cyclone_ros1_ws/src
git clone https://github.com/whgdms1230/cyclone_bridge
git clone https://github.com/whgdms1230/cyclone_ros1_node
git clone https://github.com/eclipse-cyclonedds/cyclonedds
```

다음으로 ROS 1.0(noetic) 환경에서 빌드를 수행한다.

```bash
cd ~/cyclon_ros1_ws
source /opt/ros/notic/setup.bash
colcon build
```

### 2.2 cyclone_ros2_node

cyclone_ros2_node 패키지의 설치 방법은 다음과 같다.

```bash
mkdir -p cyclone_ros2_ws/src
cd cyclone_ros2_ws/src
git clone https://github.com/whgdms1230/cyclone_bridge
git clone https://github.com/whgdms1230/cyclone_ros2_node
git clone https://github.com/eclipse-cyclonedds/cyclonedds
```

다음으로 ROS 2.0(foxy) 환경에서 빌드를 수행한다.

```bash
cd ~/cyclon_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build
```

### 2.3 실행

#### 2.3.1 Launch
cyclone_ros1_node의 실행 방법은 다음과 같다.

```bash
cd ~/cyclone_ros1_ws
source install/setup.bash

roslaunch cyclone_ros1_node cyclone_ros1_node.launch
```

cyclone_ros2_node의 실행 방법은 다음과 같다.

```bash
cd ~/cyclone_ros2_ws
source install/setup.bash

ros2 launch cyclone_ros2_node cyclone_ros2_node.xml
```

#### 2.3.2 명령 송수신

TODO

## 3. 송수신 메시지

### 3.1 Messages.idl
해당 패키지에서 사용한 `.idl` 파일인 [Messages.idl](https://github.com/whgdms1230/cyclone_bridge/blob/main/src/messages/Messages.idl)은 다음과 같다.

```c
module CycloneBridgeData
{
  struct IntNumber
  {
    uint32 int_num;
  };
  struct StrString
  {
    string messages;
  };
  struct Msg
  {
    IntNumber cnt;
    StrString messages;
  };
};
```

* `IntNumber` 메시지는 정수형 변수인 int_num 하나를 가지고 있다. 물론 정수형 변수 하나만 dds 통신하는 경우에 이러한 메시지 구조에 담을 필요가 없지만 예시를 위해 `IntNumber`라는 메시지 구조에 담았다.
* `StrString` 메시지는 string 변수인 messages 변수 하나를 가지고 있다.
* `Msg` 메시지는 위에서 선언한 `IntNumber` 변수 cnt와 `StrString` 변수 messages 를 가지고 있으며, `Msg` 메시지 구조를 이용하여 DDS 메시지 통신을 할 것이다. 

### 3.2 Messages.h와 Messages.c
[Messages.h](https://github.com/whgdms1230/cyclone_bridge/blob/main/src/messages/Messages.h)와 [Message.c](https://github.com/whgdms1230/cyclone_bridge/blob/main/src/messages/Messages.c)는 Messages.idl로부터 idlc를 이용하여 변환한 파일이다.

#### 3.2.1 Message.h
Messages.h는 Messages.idl에서 정의한 메시지에 대한 구조체 선언과 dds 통신 시 동적 할당을 위한 `alloc`, `free` 함수를 선언한 헤더파일이다.

`Msg` 각각의 메시지에 대해 선언된 구조체를 보면 다음과 같다.

```h
typedef struct CycloneBridgeData_Msg
{
  struct CycloneBridgeData_IntNumber cnt;
  struct CycloneBridgeData_StrString messages;
} CycloneBridgeData_Msg;

extern const dds_topic_descriptor_t CycloneBridgeData_Msg_desc;

#define CycloneBridgeData_Msg__alloc() \
((CycloneBridgeData_Msg*) dds_alloc (sizeof (CycloneBridgeData_Msg)));

#define CycloneBridgeData_Msg_free(d,o) \
dds_sample_free ((d), &CycloneBridgeData_Msg_desc, (o))
```

`Msg` 메시지는 `CycloneBridgeData_Msg`라는 구조체로 선언되어있으며, 그 안의 `IntNumber`, `StrString` 데이터 타입은 `CycloneBridgeData_IntNumber`, `CycloneBridgeData_StrString`라는 구조체로 선언되어 있다. 해당 부분은 생략했지만, Messages.h 파일을 보면 나머지 두 메시지 형태에 대한 구초제 선언도 확인할 수 있다.

다음으로 dds topic을 만들 때 사용되는 dds_create_topic() 함수의 인자로 사용되는 `CycloneBridgeData_Msg_desc`를 선언한다.

마지막으로 `CycloneBridgeData_Msg__alloc()` 매크로 함수와 `CycloneBridgeData_Msg_free()` 매크로 함수를 선언한다.

#### 3.2.2. Messages.c
`Msg` 메시지만 살펴보면, Messages.c에는 `CycloneBridgeData_Msg_desc`을 정의하기 위한 `CycloneBridgeData_Msg_ops`정의와 `CycloneBridgeData_Msg_desc` 정의가 구현되어있다. 해당 부분은 해당 자료구조에 대한 데이터의 크기를 정의하는 부분으로 생각되며, 자료구조의 크기가 잘못 선언되면 dds 통신 시 데이터의 손실이나 누수가 발생할 수 있다.

```c
static const uint32_t CycloneBridgeData_Msg_ops [] =
{
  /* Msg */
  DDS_OP_ADR | DDS_OP_TYPE_EXT, offsetof (CycloneBridgeData_Msg, cnt), (3u << 16u) + 7u /* IntNumber */,
  DDS_OP_ADR | DDS_OP_TYPE_EXT, offsetof (CycloneBridgeData_Msg, messages), (3u << 16u) + 7u /* StrString */,
  DDS_OP_RTS,

  /* IntNumber */
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (CycloneBridgeData_IntNumber, int_num),
  DDS_OP_RTS,

  /* StrString */
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (CycloneBridgeData_StrString, messages),
  DDS_OP_RTS
};

const dds_topic_descriptor_t CycloneBridgeData_Msg_desc =
{
  .m_size = sizeof (CycloneBridgeData_Msg),
  .m_align = sizeof (char *),
  .m_flagset = DDS_TOPIC_NO_OPTIMIZE,
  .m_nkeys = 0u,
  .m_typename = "CycloneBridgeData::Msg",
  .m_keys = NULL,
  .m_nops = 7,
  .m_ops = CycloneBridgeData_Msg_ops,
  .m_meta = ""
};
```