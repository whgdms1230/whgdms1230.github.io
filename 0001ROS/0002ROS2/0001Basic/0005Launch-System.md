---
sort: 5
---

# Launch System

## 0. 참고 문헌
*- ROS 2로 시작하는 로봇 프로그래밍(표윤석, 임태훈)*

*- [ROS 2.0 foxy Doc(Creating a launch file)](https://docs.ros.org/en/foxy/Tutorials/Launch-Files/Creating-Launch-Files.html)*

*- [ROS 2.0 foxy Doc(Using Python, XML, and YAML for ROS 2 Launch Files)](https://docs.ros.org/en/foxy/Guides/Launch-file-different-formats.html)*

## 1. Launch system
`Launch`는 복수의 노드를 함께 실행시키도록 하며, 노드 간의 메시지를 주고받을 수 있게 한다. 이 때, 노드를 실행할 때 패키지의 매개변수나 노드 이름, 노드 네임스페이스, 환경변수 변경 등을 설정할 수 있다.

## 2. Launch 파일의 종류
`ROS 2.0`에서 사용하는 `launch` 파일에는 `.launch.py` 형태와 ROS 1.0과 같은 `XML` 형태와 마지막으로 `YAML` 형태가 있다.

## 3. .launch.py 작성
다음은 foxy documentation의 [example.launch.py](https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html)를 가져왔다.
```python
# example.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    background_r_launch_arg = DeclareLaunchArgument(
        "background_r", default_value=TextSubstitution(text="0")
    )
    background_g_launch_arg = DeclareLaunchArgument(
        "background_g", default_value=TextSubstitution(text="255")
    )
    background_b_launch_arg = DeclareLaunchArgument(
        "background_b", default_value=TextSubstitution(text="0")
    )
    chatter_ns_launch_arg = DeclareLaunchArgument(
        "chatter_ns", default_value=TextSubstitution(text="my/chatter/ns")
    )

    # include another launch file
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('demo_nodes_cpp'),
                'launch/topics/talker_listener.launch.py'))
    )
    # include another launch file in the chatter_ns namespace
    launch_include_with_namespace = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(LaunchConfiguration('chatter_ns')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('demo_nodes_cpp'),
                        'launch/topics/talker_listener.launch.py'))
            ),
        ]
    )

    # start a turtlesim_node in the turtlesim1 namespace
    turtlesim_node = Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        )

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters
    turtlesim_node_with_parameters = Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim',
            parameters=[{
                "background_r": LaunchConfiguration('background_r'),
                "background_g": LaunchConfiguration('background_g'),
                "background_b": LaunchConfiguration('background_b'),
            }]
        )

    # perform remap so both turtles listen to the same command topic
    forward_turtlesim_commands_to_second_turtlesim_node = Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )

    return LaunchDescription([
        background_r_launch_arg,
        background_g_launch_arg,
        background_b_launch_arg,
        chatter_ns_launch_arg,
        launch_include,
        launch_include_with_namespace,
        turtlesim_node,
        turtlesim_node_with_parameters,
        forward_turtlesim_commands_to_second_turtlesim_node,
    ])
```

### 3.1 generate_launch_description
`launch` 파일을 작성하는데, `generate_launch_description` 메소드를 기본적으로 사용한다. 해당 메소드는 `LaunchConfiguration` 클래스를 이용하여 실행 관련 설정을 초기화하고, 리턴값으로 `LaunchDescription` 클래스를 반환한다.

### 3.2 LaunchConfiguration
`LaunchConfiguration` 클래스는 `LaunchDescription`에서 사용할 인자들을 생성하고 초기화시킨다.

### 3.3 DeclareLaunchArgument
`DeclareLaunchArgument` 클래스는 `LaunchConfiguration`으로 설정한 변수를 `launch` 인자로 선언한다.

### 3.4 Node
`Node` 클래스는 실행할 노드를 설정한다. 기본적으로 `package`, `executable`, `name`, `parameters`, `output`을 설정하며, 필요에 따라 `remappings`, `namespace`를 사용할 수 있다.

* `package` : 실행할 패키지 이름
* `executable` : 실행 가능한 노드의 이름
* `name` : 지정한 노드를 실행할 때 실제로 사용할 이름
* `parameters` : 특정 파라미터 값 또는 `DeclareLaunchArgument`에서 지정한 변수 등 사용할 파라미터(파라미터가 여러 개일 때 `{}` 단위로 구분)
* `output` : 로깅 설정. 기본적으로 특정 파일 이름(~/.ros/log/xxx/launch.log)에 로깅 정보가 기록되고, `screen`으로 지정하면 터미널 창에 출력됨.
* `remappings` : 토픽의 이름을 변경할 수 있음.
```python
...
        Node(
          ...
          remappings=[
            ('/cmd_vel', '/my_cmd_vel'),
          ]
          ...
        )
...
```

* namespace : 노드, 토픽, 서비스, 액션, 파라미터 등의 접두사 추가
```python
...
def generate_launch_description():
    my_robot = LaunchConfiguration('my_robot')

    return LaunchDescription([
        ...
        Node(
          ...
          namespace=my_robot,
          ...
        )
        ...
    ])
```

### 3.5 add_action
`Node`가 여러 개 이거나, 선언할 인자가 많아서 `return`할 인자가 많을 때, `add_action` 함수를 이용하여 `return`을 간결하게 할 수 있다.
```python
def generate_launch_description():
    ...

    launch_description = LaunchDescription()

    launch_description.add_action(launch.actions.DeclareLaunchArgument(
      ...
    ))

    first_node = Node( ... )

    second_node = Node( ... )

    launch_description.add_action(first_node)
    launch_description.add_action(second_node)

    return launch_description
```

### 3.6 IncludeLaunchDescription
다른 `.launch.py`을 불러올 때 사용하며, 다른 패키지의 `.launch.py`를 불러올 때는 `get_package_share_directory` 함수를 이용하여 불러올 수 있다.
```python
...

def generate_launch_description():

    return LaunchDescription([
        # 같은 패키지의 X.launch.py를 불러올 때
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), 'X.launch.py']),
        ),

        # 다른 패키지 B의 Y.launch.py를 불러올 때
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('B'), 'Y.launch.py']),
        ),
    ])
```

## 4. xml 형태의 launch 파일 작성
다음으로 launch.py 와 동일한 내용의 example.launch.xml 파일을 가져왔으며, 해당 내용은 [ROS 1.0의 launch 파일](~/0001ROS1/0001Basic/0005Launch-System.html)과 동일한 형태로 구성되어 있다.

```xml
<!-- example.launch.xml -->

<launch>

  <!-- args that can be set from the command line or a default will be used -->
  <arg name="background_r" default="0"/>
  <arg name="background_g" default="255"/>
  <arg name="background_b" default="0"/>
  <arg name="chatter_ns" default="my/chatter/ns"/>

  <!-- include another launch file -->
  <include file="$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener.launch.py"/>
  <!-- include another launch file in the chatter_ns namespace-->
  <group>
    <!-- push-ros-namespace to set namespace of included nodes -->
    <push-ros-namespace namespace="$(var chatter_ns)"/>
    <include file="$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener.launch.py"/>
  </group>

  <!-- start a turtlesim_node in the turtlesim1 namespace -->
  <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1"/>
  <!-- start another turtlesim_node in the turtlesim2 namespace
      and use args to set parameters -->
  <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2">
    <param name="background_r" value="$(var background_r)"/>
    <param name="background_g" value="$(var background_g)"/>
    <param name="background_b" value="$(var background_b)"/>
  </node>
  <!-- perform remap so both turtles listen to the same command topic -->
  <node pkg="turtlesim" exec="mimic" name="mimic">
    <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
    <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
  </node>
</launch>
```
### 4.1 arguments
launch.xml에서는 다음과 같이 파일의 첫 부분에 `arg` 태그로 인자를 정의할 수 있다. 해당 인자는 이후에 가져와서 사용할 수 있으며, 상위 launch 파일이나 commandline 에서 설정이 되지 않으면 default 값으로 설정된다.
```xml
  <!-- args that can be set from the command line or a default will be used -->
  <arg name="background_r" default="0"/>
  <arg name="background_g" default="255"/>
  <arg name="background_b" default="0"/>
  <arg name="chatter_ns" default="my/chatter/ns"/>
```

### 4.2 include
`include` 태그는 launch 파일을 불러와 실행시키는 명령어로, 해당 launch파일에 정의되어 있는 명령들을 수행한다.
```xml
  <!-- include another launch file -->
  <include file="$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener.launch.py"/>
```

### 4.3 group
`group` 태그는 동일한 namespace 아래에 여러개의 노드를 묶어서 실행시킬 때 사용되는 태그이다. 
```xml
  <!-- include another launch file in the chatter_ns namespace-->
  <group>
    <!-- push-ros-namespace to set namespace of included nodes -->
    <push-ros-namespace namespace="$(var chatter_ns)"/>
    <include file="$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener.launch.py"/>
  </group>

```

### 4.4 node
`node` 태그는 특정 패키지의 하나의 특정 노드를 실행시키는 명령어이다. `node` 태그 아래에 `param` 명령을 통해서 노드안에서 사용되는 파라미터들의 설정값을 미리 설정할 수 있으며, `remap`명령을 통해 특정 토픽명을 원하는 토픽 이름으로 바꿀 수 있다.

```xml
  <!-- start a turtlesim_node in the turtlesim1 namespace -->
  <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1"/>
  <!-- start another turtlesim_node in the turtlesim2 namespace
      and use args to set parameters -->
  <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2">
    <param name="background_r" value="$(var background_r)"/>
    <param name="background_g" value="$(var background_g)"/>
    <param name="background_b" value="$(var background_b)"/>
  </node>
  <!-- perform remap so both turtles listen to the same command topic -->
  <node pkg="turtlesim" exec="mimic" name="mimic">
    <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
    <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
  </node>
```

### 4.5 이외의 xml 파일의 태그
* `<launch>` : roslaunch 구분의 시작과 끝을 나타냄
* `<node>` : 노드 실행에 대한 태그. 패키지(`pkg`), 실행 시 노드명(`name`), 패키지 내 실행 파일의 이름(`type`, `exec`), 로그 출력(`output`) 등을 설정
* `<machine>` : 노드를 실행하는 PC의 이름. address, ros-root, ros-package-path 등 설정
* `<include>` : 다른 패키지나 같은 패키지에 속해 있는 다른 launch를 불러옴
* `<remap>` : 노드 이름, 토픽 이름 등의 노드에서 사용 중인 ROS 변수의 이름 변경
* `<env>` : 경로, IP 등의 환경변수를 설정
* `<param>` : 파라미터 이름, 타이프, 값 등을 설정
* `<rosparam>` : rosparam 명령과 같이, load, dump, delete 등 파라미터 정보를 확인 및 수정
* `<group>` : 노드 그룹화
* `<test>` : 노드 테스트할 때 사용
* `<arg>` : launch 파일 내 변수 정의

## 5. YAML 파일 형태의 launch 파일
다음으로 동일한 내용의 launch 파일로, yaml 파일의 형식으로 선언되어 있는 파일이다.

해당 내용은 예제 파일의 형태를 가지고 동일한 형태로 사용해보면 좋을 것 같음.
```yaml
# example.launch.yaml

launch:

# args that can be set from the command line or a default will be used
- arg:
    name: "background_r"
    default: "0"
- arg:
    name: "background_g"
    default: "255"
- arg:
    name: "background_b"
    default: "0"
- arg:
    name: "chatter_ns"
    default: "my/chatter/ns"


# include another launch file
- include:
    file: "$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener.launch.py"

# include another launch file in the chatter_ns namespace
- group:
    - push-ros-namespace:
        namespace: "$(var chatter_ns)"
    - include:
        file: "$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener.launch.py"

# start a turtlesim_node in the turtlesim1 namespace
- node:
    pkg: "turtlesim"
    exec: "turtlesim_node"
    name: "sim"
    namespace: "turtlesim1"

# start another turtlesim_node in the turtlesim2 namespace and use args to set parameters
- node:
    pkg: "turtlesim"
    exec: "turtlesim_node"
    name: "sim"
    namespace: "turtlesim2"
    param:
    -
      name: "background_r"
      value: "$(var background_r)"
    -
      name: "background_g"
      value: "$(var background_g)"
    -
      name: "background_b"
      value: "$(var background_b)"

# perform remap so both turtles listen to the same command topic
- node:
    pkg: "turtlesim"
    exec: "mimic"
    name: "mimic"
    remap:
    -
        from: "/input/pose"
        to: "/turtlesim1/turtle1/pose"
    -
        from: "/output/cmd_vel"
        to: "/turtlesim2/turtle1/cmd_vel"
```