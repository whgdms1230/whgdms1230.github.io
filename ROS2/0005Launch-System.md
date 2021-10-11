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
다음은 turtlebot3의 [navigation2.launch.py](https://github.com/ROBOTIS-GIT/turtlebot3/blob/foxy-devel/turtlebot3_navigation2/launch/navigation2.launch.py)를 가져왔다.
```bash
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'turtlebot3_world.yaml'))

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
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
```bash
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
```bash
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
```bash
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
```bash
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