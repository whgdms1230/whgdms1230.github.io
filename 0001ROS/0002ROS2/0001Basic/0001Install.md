---
sort: 1
---

# Install

> foxy 버전은 [ubuntu 20.04](http://releases.ubuntu.com/20.04/)에서 사용되므로, 버전에 유의할 것.

## 0. 참고 문헌
*[ROS 2.0 foxy 설치 페이지(debian 패키지 이용)](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)*

## 1. Locale 설정
설치를 위해 설정된 `locale`이 `UTF-8`을 지원애햐 한다. 따라서 `locale`을 다음과 같이 설정한다.
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## 2. Source 추가
`curl` 명령을 이용하여 `gpg key`를 가져온다.

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

그 이후, `sources.list.d`에 `ROS 2.0` 설치하기 위해 `repository`를 추가한다.
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 3. 설치
설치하기 전, `debian package`를 업데이트 한 후, `foxy` 버전을 설치한다.(풀버전 설치)
```bash
sudo apt update
sudo apt install ros-foxy-desktop
sudo apt install ros-foxy-ros-base
```

## 4. 개발 및 ROS tool 설치

`ROS 2.0` 개발에 필요한 패키지들을 설치한다.
```bash
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev
```

작업 공간을 만들고, 저장소들을 불러온다.
```bash
mkdir -p ~/ros2_foxy/src
cd ~/ros2_foxy
wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
vcs import src < ros2.repos
```

`rosdep` 명령어를 이용하여 의존성 패키지들을 설치한다.
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"
```

작업공간에 설치된 패키지들을 빌드하면, `ROS` 작업에 필요한 기본 패키지들을 설치할 수 있다.
```bash
cd ~/ros2_foxy/
colcon build --symlink-install
```

## 5. 설치 확인
설치 확인을 위해 다음과 같은 예제를 실행한다.

* C++ talker
터미널창을 열어 `C++ talker`를 실행한다.
```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
```

* Python listener
또 다른 터미널창을 열어 `Python listener`를 실행한다.
```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py listener
```

그 결과, `talker`에서 `publish`하는 `message`를 `listener`의 터미널창에서 `subscribe`하는 것을 볼 수 있다.