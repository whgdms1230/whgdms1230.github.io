---
sort: 9
---

# Interface Package

## 0. 참고 문헌
*- ROS 2로 시작하는 로봇 프로그래밍(표윤석, 임태훈)*

## 1. ROS 2 인터페이스
ROS 노드 간에 데이터를 주고받기 위해 사용되는 토픽, 서비스, 액션의 데이터 형태를 ROS 2 인터페이스라고 한다. ROS 인터페이스에는 IDL(Interface Definition Language)와 msg, srv, action이 있다. 토픽, 서비스, 액션은 각각 msg, srv, action 인터페이스를 사용한다.

각 인터페이스 파일(`.msg`, `.srv`, `.action`)을 정의할 때 다음과 같이 정의한다.
```
fieldtype1 fieldname1
fieldtype2 fieldname2
fieldtype3 fieldname3
```

fieldtype은 다음과 같은 단순 자료형을 사용한다.

Type name|C++              |Python           |DDS type          |
---------|-----------------|-----------------|------------------|
bool     |bool             |builtins.bool    |boolean           |
byte     |uint8_t          |builtins.bytes   |octet             |
char     |char             |builtins.str     |char              |
float32  |float            |builtins.float   |float             |
float64  |double           |builtins.float   |double            |
int8     |int8_t           |builtins.int     |octet             |
uint8    |uint8_t          |builtins.int     |octet             |
int16    |int16_t          |builtins.int     |short             |
uint16   |uint16_t         |builtins.int     |unsigned short    |
int32    |int32_t          |builtins.int     |long              |
uint32   |uint32_t         |builtins.int     |unsigned long     |
int64    |int64_t          |builtins.int     |long long         |
uint64   |uint64_t         |builtins.int     |unsigned long long|
string   |std::string      |builtins.str     |string            |
wstring  |std::u16string   |builtins.str     |wstring           |

## 2. 인터페이스 파일의 형태
### 2.1 msg
메시지 파일은 토픽에서 사용될 메시지를 정의한 것으로 메시지에 담아 보낼 자료의 형태와 이름을 정의한다.

다음은 `geometry_msgs/msgs/Twist`의 메시지 구조이다.
```
Vector3 linear
Vector3 angular
```
여기서 Vector3 자료형은 `geometry_msgs/msgs/Vecotr3`에 정의되어있으며, 그 내용은 다음과 같다.
```
float64 x
float64 y
float64 z
```

### 2.2 srv
서비스 파일은 서비스에 사용될 자료 형태를 정의한 것으로 `---` 구분자를 이용하여 request와 response를 구분하는데 사용된다.

예를들어 다음과 같은 srv 파일이 정의되어 있다면, request로 name을 요청하고, response로 id를 회신하는 서비스가 될 것이다.
```
string name
---
string id
```

### 2.3 action
액션 파일은 액션에 사용될 자료 형태를 정의한 것으로 `---` 구분자를 이용하여 goal, result, feedback을 구분하는데 사용된다.

에를들어 다음과 같은 action 파일이 정의되어 있다면, goal로 x,y 좌표를 보내고, result로 성공 여부를 보내며, feedback으로 현재의 위치를 보내는 액션이 될 것이다.
```
float64 x
float64 y
---
bool success
---
float64 cur_x
float64 cur_y
```

## 3. 인터페이스 패키지
인터페이스 패키지는 사용자가 사용하고자 하는 인터페이스 형태를 사용하기 위하여 작성하는 패키지이다. 일반적으로 인터페이스로만 구성되어 있는 독립된 패키지를 만들어 사용하는데, 이는 의존성면에서 관리하기 용이하기 때문에 인터페이스 패키지를 만들어 관리를 한다.

### 3.1 인터페이스 패키지 만들기
인터페이스 패키지에는 각 인터페이스를 정의한 폴더로 구성이 되어있으며, 해당 인터페이스 폴더 아래에 해당하는 인터페이스 파일들이 저장되어있다.
```
interface_package
├── action
│   └── interface_action.action
├── msg
│   └── interface_msg.msg
├── srv
│   └── interface_srv.srv
├── package.xml
└── CMakeLists.txt
```

### 3.2 Package.xml
패키지 설정 파일에는 빌드 시에 DDS에서 사용되는 IDL 생성과 관련한 `rosidl_default_generators`가 사용되고, 실행 시 `builtin_interfaces`와 `rosidl_default_runtime`이 사용되므로, 해당 내용을 추가해야 한다.
```xml
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>builtin_interfaces</exec_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
```

### 3.3 CMakeLists.txt
빌드 설정파일에는 `find_package()`로 `builtin_interfaces`와 `rosidl_default_generators`를 가져온다. 또한, `set` 명령어로 `msg`, `srv`, `action` 파일을 지정하고, `rosidl_generate_interfaces`에 해당 `set`을 입력한다.

```cmake
cmake_minimum_required(VERSION 3.5)
project(test_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg
  "msg/interface_msg.msg"
)

set(srv
  "srv/interface_srv.srv"
)

set(action
  "action/interface_action.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg}
  ${srv}
  ${action}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```