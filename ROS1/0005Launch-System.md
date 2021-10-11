---
sort: 5
---

# Launch System

## 0. 참고 문헌
*- ROS 로봇 프로그래밍(표윤석, 조한철, 정려운, 임태훈)*

*- [ROS wiki (roslaunch / Commandline Tools)](http://wiki.ros.org/roslaunch/Commandline%20Tools)*

*- [ROS wiki (roslaunch / XML)](http://wiki.ros.org/roslaunch/XML)*

## 1. Launch system
`Launch`는 복수의 노드를 함께 실행시키도록 하며, 노드 간의 메시지를 주고받을 수 있게 한다. 이 때, 노드를 실행할 때 패키지의 매개변수나 노드 이름, 노드 네임스페이스, 환경변수 변경 등을 설정할 수 있다.

## 2. Launch 파일의 종류
`ROS 1.0`에서는 `XML` 형태를 기반으로 한 `.launch` 파일을 사용한다.

## 3. Launch 태그
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