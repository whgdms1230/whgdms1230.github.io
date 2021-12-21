---
sort: 3
---

# CMakeLists.txt

## 0. 참고 문헌
*- [CmakeLists.txt ros wiki page](http://wiki.ros.org/catkin/CMakeLists.txt)*

*- [CMake Main page](https://cmake.org/)*

## 1. CMakeLists.txt란?

해당 페이지는 `CmakeLists.txt ros wiki page`의 내용을 주로 옮겨왔다.

`CMakeLists.txt`파일은 소프트웨어 패키지 빌드를 위한 `CMake` 빌드 시스템의 입력 파일이다. `CMake`와 호환되는 패키지는 하나 이상의 `CMakeLists.txt` 파일을 가지며, 이 파일을 통해 어떻게 패키지 내의 코드를 빌드하고 어디에 설치할 지를 기술하게 된다.

## 2. CMake 버전
패키지를 빌드하기 위해 최소로 필요한 `CMake` 버전을 입력한다. `ROS 1.0`의 `catkin`에서는 최소 2.8.3 이상을 필요로 한다.
```cmake
cmake_minimum_required(VERSION 2.8.3)
```

## 3. Package Name
패키지 명을 project 함수를 통해 전달한다. 현재 패키지의 폴더 명과 동일해야 한다.
```cmake
project([PACKAGE_NAME])
```

## 4. 빌드에 필요한 CMake 패키지 찾기
`find_package()` 함수를 이용하여 패키지 빌드를 위해 필요한 다른 의존 패키지들을 입력한다.

`ROS 1.0`의 경우 `catkin` 패키지가 필요하므로 항상 `catkin` 패키지를 추가해야 한다.
```cmake
find_package(catkin REQUIRED)
```

또한 `COMPONENTS` 인자를 통해 해당 패키지에서 가져올 부분을 다음 예시처럼 명시해주어야 한다.
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  ...
)
```
이렇게 하면, 해당 패키지에 대한 헤더 파일 경로, 라이브러리 정보 등이 `catkin_` 환경 변수에 적용될 것이다. 즉 `catkin_INCLUDE_DIRS` 안에 각 `catkin` 패키지에 대한 헤더 파일 경로 뿐 아니라, 각 패키지에 대한 환경변수도 생성이 된다. 이렇게 해두면 나중에 편리할 수 있다.

물론 각각의 패키지에 대해서 `find_package()` 함수를 사용할 수 있으며, 각 패키지에 대한 `[PACKAGE_NAME]_INCLUDE_DIRS`, `[PACKAGE_NAME]_LIBRARIES` 등과 같이 별도의 환경변수 세트가 생성된다.

그 이외의 `catkin` 패키지가 아닌 패키지들을 추가적으로 필요한 다른 의존성 패키지들을 추가하면 된다. 예를 들어 `Boost` 패키지를 사용할 때, 다음과 같이 패키지를 추가해주어야 한다.
```cmake
find_package(Boost REQUIRED COMPONENTS
  system
)
```

## 5. 파이썬 모듈 지원 활성화
`ROS` 패키지에 파이썬 모듈이 포함되는 경우(즉, `rospy`를 사용하는 경우)에 사용된다. `setup.py` 파일이 필요하며, 아래와 같은 호출이 `CMakeLists.txt`에 추가되어야 하며, 그 위치는 `generate_messages()`와 `catkin_package()` 호출 전이어야 한다.
```cmake
catkin_python_setup()
```

## 6. 메세지, 서비스, 액션 타겟
`ROS`의 메시지 파일(`.msg`), 서비스 파일(`.srv`), 액션 파일(`.action`)을 사용하려면 패키지 빌드 전에 이들 파일을 위한 특별한 전처리 빌드 단계가 필요하다. 매크로를 통해 사용하는 프로그래밍 언어에 적합한 형태로 메시지, 서비스, 액션을 전처리하게 된다. 빌드 시스템은 `gencpp`, `genpy`, `genlisp` 등의 생성 도구를 이용하여 바인딩을 위한 전처리를 수행한다. 아래와 같이 매시지, 서비스, 액션 각각을 위한 세 개의 매크로가 제공된다.
* `add_message_files()`
* `add_service_files()`
* `add_action_files()`

ROS 메세지, 서비스, 액션을 생성하기 전에 먼저 `find_package()` 에서 `message_generation`을 불러와야 한다.
```cmake
find_package(catkin REQUIRED COMPONENTS
  message_generation
)
```
그 이외에도 메세지, 서비스, 액션 파일에 `std_msgs`나 `geometry_msgs`를 사용한다면, `find_package()`에 추가해야 한다.
```cmake
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)
```

그 이후에, `add_message_files()`, `add_service_files()`, `add_action_files()` 매크로를 이용해, 메세지, 서비스, 액션 파일을 추가한다. 여기서 `FILES` 인자는 패키지 폴더의 각각 `msg`, `srv`, `action` 폴더 아래의 `.msg`, `.srv`, `.action` 파일들을 참조하도록 한다.
```cmake
# 메세지 추가
add_message_files(
   FILES
   [MessageName_1.msg]
   [MessageName_2.msg]
   [MessageName_3.msg]
   ...
)
# 서비스 추가
add_service_files(
   FILES
   [ServiceName_1.srv]
   [ServiceName_2.srv]
   [ServiceName_3.srv]
   ...
)
# 액션 추가
add_action_files(
   FILES
   [ActionName_1.action]
   [ActionName_2.action]
   [ActionName_3.action]
   ...
)
```

매크로 이후에 `generate_message()` 함수를 이용하여 메세지를 생성한다. 이 때, 의존성으로 필요한 `ROS` 표준 메세지 타입이 있다면 추가해주어야 한다.
```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
```

`generate_message()`를 수행한 다음에 `catkin_package()`를 수행해야 하는데, `catkin_package()`에서 `CATKIN_DEPENDS`로 `message_runtime`을 추가해야 한다.
```cmake
catkin_package(
 ...
 CATKIN_DEPENDS message_runtime ...
 ...
)
```

## 7. catkin_package()
`catkin_package()` 는 `catkin`이 제공하는 `CMake` 매크로 함수로 `catkin` 관련 정보를 빌드 시스템에 전달하여 `pkg-config`와 `CMake` 파일을 생성하기 위해 필요하다. 이 함수는 반드시 `add_library()` or `add_executable()`로 빌드 타겟을 선언하기 전에 호출하여야 하며, 다음의 5 개의 선택가능한 인자를 가진다. 주로 `CATKIN_DEPENDS`를 사용하는 것 같다.

* `INCLUDE_DIRS` - 패키지 내부 폴더인 `include`의 헤더 파일을 사용하겠다는 설정
* `LIBRARIES` - 사용할 의존성 라이브러리를 설정
* `CATKIN_DEPENDS` - `roscpp`나 `std_msgs`와 같이 프로젝트가 의존성을 가진 다른 `catkin` 프로젝트
* `DEPENDS` - 프로젝트가 의존성을 가진 `Non-catkin CMake` 프로젝트
* `CFG_EXTRAS` - 기타 설정 옵션

## 8. 빌드 타겟 정의
빌드 타겟은 일반적으로 아래 두 가지 방법 중 하나를 선택한다.
* 실행 파일 타겟 - 실행할 수 있는 프로그램 형태
* 라이브러리 타겟 - 빌드나 실행시 실행 파일 타겟이 사용할 라이브러리

### 8.1 타겟 설정
*우선 `catkin에서 빌드 타겟의 이름은 어느 폴더에 빌드/설치되느냐와 관계 없이 중복되지 않는 유일한 것이어야 한다. 이는 CMake의 규칙이다.* 타겟의 이름을 변경하고자 할때에는 아래와 같이 `set_target_properties()` 함수를 사용하면 된다.
```cmake
set_target_properties(rviz_image_view
                      PROPERTIES OUTPUT_NAME image_view
                      PREFIX "")
```
이 명령은 타겟의 이름이 `rviz_image_view`에서 `image_view`로 변경되어 빌드/설치 결과물에 반영된다.

### 8.2 출력 디렉토리 설정
실행 파일과 라이브러리를 위한 기본 출력 디렉토리가 정해져 있으나 특별한 경우 이를 원하는 대로 수정할 수 있다. 예로 파이썬 바인딩이 필요한 라이브러리의 경우 파이썬 `import`가 가능한 폴더에 위치해야 할 수 있다.
```cmake
set_target_properties(python_module_library
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
```

### 8.3 Include 경로와 Library 경로
타겟을 정의하기 전에 빌드에 필요한 헤더 파일이나 라이브러리 등의 경로에 대한 정보를 기술해 두어야 한다.
* `Include` 경로 - 코드 빌드를 위해 필요한 헤더 파일의 위치 지정
* `Library` 경로 - 실행 파일 타겟 빌드를 위해 필요한 라이브러리의 위치 지정

#### 8.3.1 include_directories()
`include_directories()`의 인자는 앞서 `find_package()` 호출 시 생성된 `*_INCLUDE_DIRS` 환경 변수와 추가로 지정한 디렉토리 경로가 반영된다. 예를 들어, `catkin`과 `Boost`를 사용하는 경우, 아래와 같이 기술하면 된다. 여기서 맨 앞의 `include`는 패키지 내부의 `include` 폴더를 나타내며, 해당 경로를 추가해주어야 패키지 내부의 헤더파일들을 추가할 수 있다.
```cmake
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
```

#### 8.3.2 link_directories()
라이브러리 경로를 추가하기 위해 `CMake`의 `link_directories()` 함수를 사용할 수 있으나, 추천되는 방법은 아니다. 왜냐하면 `find_package()` 실행 시 모든 `catkin`과 `CMake` 패키지를 위한 `link` 정보를 얻게 되기 때문이다. `target_link_libraries()`를 이용하여 라이브러리를 링크하면 된다.
```cmake
link_directories(~/my_libs)
```

### 8.4 실행 파일 추가
빌드 후 생성할 실행 파일을 추가하기 위해 `add_executable()` 함수를 사용한다. 다음 예시는 `src/main.cpp`, `src/some_file.cpp`, `src/another_file.cpp`을 참조하여 `myProgram`이라는 실행파일을 생성한다.
```cmake
add_executable(myProgram src/main.cpp src/some_file.cpp src/another_file.cpp)
```

### 8.5 라이브러리 타겟 추가
`add_library()` 함수를 이용하여 빌드 후 생성할 라이브러리를 선언한다. 기본적으로 `catkin`은 공유 라이브러리로 빌드한다. 다음 예시는 현재 패키지에 포함되어있는 소스 코드들을 참조하여 `PROJECT_NAME`의 이름으로 라이브러리를 생성한다.
```cmake
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
```

### 8.6 target_link_libraries()
`target_link_libraries()` 함수로 실행 파일 타겟과 링크될 라이브러리를 지정한다. 일반적으로 `add_executable()` 또는 `add_library()` 호출 이후에 위치하며, 만약 `ros is not found` 라는 오류가 확인되면 `${catkin_LIBRARIES}`를 추가한다.

```cmake
target_link_libraries(<executableTargetName>, <lib1>, <lib2>, ... <libN>)
```
여기서 `<executableTargetName>`은 `add_executable()` 또는 `add_library()` 함수의 첫 번째 인자(`executable name` or `library name`)과 같다.

### 8.7 add_dependencies()
빌드하려는 패키지가 타겟하는 의존성 패키지에서 `messages/services/actions`을 사용한다면, `catkin_EXPORTED_TARGETS`에 대한 의존성을 추가해주어야 한다.(예시에서 `some_target`은 `add_executable()`로 설정한 실행 파일 타겟 이름이다)
```cmake
add_dependencies(some_target ${catkin_EXPORTED_TARGETS})
```

또한 빌드하려는 패키지가 `message/services/actions`을 사용한다면 역시 다음과 같이 의존성을 추가해주어야 한다.
```cmake
add_dependencies(some_target ${${PROJECT_NAME}_EXPORTED_TARGETS})
```

만약 두 가지의 경우 모두 해당한다면, 다음과 같이 사용하며, 대부분의 `ROS` 패키지에서는 해당 함수를 사용하게 된다.
```cmake
add_dependencies(some_target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

## 9. install
`workspace` 내의 `build` 영역에 설치할 디렉토리를 설정한다. `DIRECTORY` 인자 뒤에 설치할 디렉토리들을 선정하고, `DESTINATION` 인자 뒤에는 설치 경로, 즉 공유할 경로를 설정하면 된다.
```cmake
install(DIRECTORY launch rviz ...
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

실행 가능한 `Python` 스크립트를 설치하는 경우에는 `catkin_install_python()`을 사용한다.
```cmake
catkin_install_python(PROGRAMS scripts/test_script
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```