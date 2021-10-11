---
sort: 1
---

# Install

> noetic 버전은 [ubuntu 20.04](http://releases.ubuntu.com/20.04/)에서 사용되므로, 버전에 유의할 것.

## 0. 참고 문헌
*- [ROS 1.0 설치 관련 위키 페이지](http://wiki.ros.org/Installation/Ubuntu)*

## 1. source.list 설정
쉘 스크립트를 이용. /etc/apt/source.list.d에 ros-latest.list를 추가하여 ROS 패키지의 `binary release`를 받을 수 있는 저장소를 추가.
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

## 2. key 설정
`curl` 설치 (curl 설치가 되어있다면 넘어가도 된다.)
> 여기서 curl은 서버와 통신할 수 있는 커맨드 툴로서, 다양한 프로토콜을 지원한다.

```bash
sudo apt install curl
```
curl 명령을 통해 가져온 gpg key를 apt-key 명령을 통해 추가한다.
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

## 3. 설치
설치하기 전, `debian package`를 업데이트 한 후, `noetic` 버전을 설치한다.(풀버전 설치)
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```

## 4. 설치 확인
설치가 되었는지 확인
```bash
apt search ros-noetic
```