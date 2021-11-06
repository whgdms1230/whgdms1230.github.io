---
sort: 2
---

# Catkin

## 0. 참고 문헌
*- Programming Robots with ROS(Morgan Quigley, Brian Gerkey, William D. Smart)*

*- [catkin overview ros wiki pages](https://wiki.ros.org/catkin/conceptual_overview)*

## 1. Catkin 이란?
`catkin`은 `ROS`에 사용되는 실행 프로그램, 라이브러리, 스크립트 및 다른 코드에서 사용할 인터페이스를 생성할 수 있도록 하는 `ROS` 빌드 시스템이다. `catkin`은 `CMake` 매크로들과 일반적인 `CMake` workflow에 추가적인 기능을 제공하기 위한 전용 파이썬 스크립트로 구성된다.

## 2. ROS 프로젝트를 진행하며 자주 사용한 catkin CLI
다음은 `ROS` 프로젝트를 진행하면서 주로 사용되는 `command line arguments`를 정리하였다.

* `catkin_create_pkg [package_name] [dependency_package1] [dependency_package2] ...`
`ROS` 패키지를 생성하는 명령으로, `package_name`으로 패키지를 생성하며, `CMakeLists.txt`와 `package.xml`을 포함한 패키지 폴더를 생성한다. 이 때, 패키지 이름은 모두 소문자를 사용해야하며, 공백이 있으면 안된다.

* `catkin_make`
`catkin` 빌드 명령어로, `workspace` 위치에서 해당 명령어로 `workspace` 내부에 있는 패키지들을 빌드할 수 있다.