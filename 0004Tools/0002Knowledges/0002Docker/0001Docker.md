---
sort: 1
---

# Docker

## 참고 문헌
*- [Docker](https://www.docker.com/)*

*- [Docker redhat](https://www.redhat.com/ko/topics/containers/what-is-docker)*

*- [Docker amazon](https://aws.amazon.com/ko/docker/)*

*- [Docker ROS](https://hub.docker.com/_/ros)*

*- [ROS Wiki - Docker](http://wiki.ros.org/docker/Tutorials/Docker)*

*- [SLAMOps를 위한 첫걸음 - Docker + CI](https://www.cv-learn.com/20210808-docker-for-slam/)*

*- [[Docker] 개념 정리 및 사용방법까지](https://cultivo-hy.github.io/docker/image/usage/2019/03/14/Docker%EC%A0%95%EB%A6%AC/)*

## Docker 개념
### Docker

도커(Docker)는 컨테이너 기반의 오픈소스 가상화 플랫폼으로, 컨테이너를 관리하는 툴이다.

도커를 사용하는 가장 큰 이유는 다양한 프로그램, 실행환경을 컨테이너로 추상화하고 동일한 인터페이스를 제공하여 프로그램의 배포 및 관리를 쉽게하기 위해서이다.

### Container

컨테이너(Container)는 개별 Software의 실행에 필요한 실행환경을 독립적으로 운용할 수 있도록 기반환경 또는 다른 실행환경과의 간섭을 막고 실행의 독립성을 확보해주는 운영체계 수준의 격리 기술을 말한다.

### Image

이미지(Image)는 컨테이너 실행에 필요한 파일과 설정값등을 포함하고 있는 것으로 상태값을 가지지 않고 변하지 않는다(Immutable). 컨테이너는 이미지를 실행한 상태라고 볼 수 있고 추가되거나 변하는 값은 컨테이너에 저장된다. 즉, 실행된 컨테이너 내에서 변경사항이 발생하면 이미지에는 영향이 없으며, 컨테이너 내에서만 변경사항이 적용된다.

또한 이미지 하나로 여러개의 컨테이너를 생성할 수 있고 컨테이너의 상태가 바뀌거나 컨테이너가 삭제되더라도 이미지는 변하지 않고 그대로 남아있는다.

### Docker Layer

이미지는 컨테이너를 실행하기 위한 모든 정보를 가지고 있기 때문에 보통 용량이 크다. 처음 이미지를 다운받을 땐 크게 부담이 안되지만 기존 이미지에 파일 하나 추가했다고 큰 용량의 이미지를 다시 다운받는 것은 비효율적이다.

도커는 이런 문제를 해결하기 위해 레이어(layer)라는 개념을 사용하고 유니온 파일 시스템을 이용하여 여러개의 레이어를 하나의 파일시스템으로 사용할 수 있게 해준다. 이미지는 여러개의 읽기 전용 read only 레이어로 구성되고 파일이 추가되거나 수정되면 새로운 레이어가 생성된다.

### 이미지 경로

이미지는 `url` 방식으로 관리하며 태그를 붙일 수 있다. `ubuntu 14.04` 이미지는 `docker.io/library/ubuntu:14.04` 또는 `docker.io/library/ubuntu:trusty` 이고 `docker.io/library`는 생략가능하여 `ubuntu:14.04` 로 사용할 수 있다. 이러한 방식은 이해하기 쉽고 편리하게 사용할 수 있으며 태그 기능을 잘 이용하면 테스트나 롤백도 쉽게 할 수 있다.

### Dockerfile

도커는 이미지를 만들기 위해 Dockerfile이라는 파일에 자체 `DSLDomain-specific language`언어를 이용하여 이미지 생성 과정을 적는다.

## Docker 설치

* [Docker 설치 페이지](https://docs.docker.com/engine/install/debian/)

```bash
# curl 설치 안되어있다면 설치
sudo apt-get install curl

curl -fsSL https://get.docker.com/ | sudo sh 
```

설치 확인하기

```bash
sudo docker run hello-world
```

## Docker 기본 명령어

* [Use the Docker command line](https://docs.docker.com/engine/reference/commandline/cli/)

* 컨테이너 목록 확인하기 (ps)

```bash
docker ps [OPTIONS]
```

* 컨테이너 중지하기 (stop)

```bash
docker stop [OPTIONS] CONTAINER [CONTAINER...]
```

> 도커 ID의 전체 길이는 64자리이나, 명령어의 인자로 전달할 때는 전부 입력하지 않아도 된다. 예를 들어 ID가 abcdefgh…라면 abcd만 입력해도 된다.

* 컨테이너 제거하기 (rm) : 종료된 컨테이너를 완전히 제거하는 명령

```bash
docker rm [OPTIONS] CONTAINER [CONTAINER...]
```

* 이미지 목록 확인하기 (images)

```bash
docker images [OPTIONS] [REPOSITORY[:TAG]]
```

* 이미지 다운로드하기 (pull)

```bash
docker pull [OPTIONS] NAME[:TAG|@DIGEST]
```

* 이미지 삭제하기 (rmi)

```bash
docker rmi [OPTIONS] IMAGE [IMAGE...]
```

> images 명령어를 통해 얻은 이미지 목록에서 이미지 ID를 입력하면 삭제된다. 단, 컨테이너가 실행중인 이미지는 삭제되지 않는다.

* 컨테이너 로그 보기 (logs) : 컨테이너가 정상적으로 동작하는지 확인하기 위함

```bash
docker logs [OPTIONS] CONTAINER
```

* 컨테이너 명령어 실행하기 (exec) : 실행중인 컨테이너에 들어가거나 컨테이너의 파일을 실행하는 경우.

```bash
docker exec [OPTIONS] CONTAINER COMMAND [ARG...]
```

* 컨테이너 실행 (run)

```bash
docker run [OPTIONS] IMAGE [COMMAND] [ARG...]
```

### run 명령어

* 출처 : [docker run 커맨드 사용법](https://www.daleseo.com/docker-run/)

* -d 옵션 : `-d` 옵션을 사용하면 컨테이너가 detached 모드에서 실행되며, 실행 결과로 컨테이너 ID만을 출력한다.

* -it 옵션 : `-i` 옵션과 `-t` 옵션은 같이 사용한 경우로, 이 두 옵션은 컨테이너를 종료하지 않은체로, 터미널의 입력을 계속해서 컨테이너로 전달하기 위해서 사용한다. 따라서, 컨테이너의 쉘(shell)이나 CLI 도구를 사용할 때 유용하게 사용된다.

* --name 옵션 : 컨테이너에 이름을 부여하는 옵션으로 해당 이름으로 컨테이너를 식별할 수 있다.

* -e, --env 옵션 : Docker 컨테이너의 환경변수를 설정하며, `-e` 옵션은 Dockerfile의 ENV 설정도 덮어쓴다.

* --ip 옵션 : Docker container에 특정 IP를 할당해야 하는 경우 --ip 옵션으로 IP를 할당할 수 있다.

* -p 옵션 : 호스트와 컨테이너 간의 포트(port) 배포(publish)/바인드(bind)를 위해서 사용되어 호스트(host) 컴퓨터에서 컨테이너에서 리스닝하고 있는 포트로 접속할 수 있도록 설정해준다.

* --entrypoint 옵션 : Dockerfile의 ENTRYPOINT 설정을 덮어쓰기 위해서 사용

* --rm 옵션 : 컨테이너를 일회성으로 실행할 때 사용되며, 컨테이너가 종료될 때 컨테이너와 관련된 리소스(파일 시스템, 볼륨)까지 깨끗이 제거해준다.

## 기본 이미지 다운 및 컨테이너 작업

도커는 이미지를 만들기 위해 컨테이너의 상태를 그대로 이미지로 저장하는 방법을 사용한다.

어떤 애플리케이션을 이미지로 만든다면 리눅스만 설치된 컨테이너에 애플리케이션을 설치하고 그 상태를 그대로 이미지로 저장합니다.

* [우분투 이미지 다운로드](https://hub.docker.com/_/ubuntu)

```bash
docker pull ubuntu:20.04
```

* [ROS 이미지 다운로드](https://registry.hub.docker.com/_/ros/)

```bash
docker pull ros:foxy-ros-core
```

## Dockerfile 만들기

* [[Docker] Dockerfile 개념 및 작성법](https://wooono.tistory.com/123)


* FROM : 베이스 이미지
> 어느 이미지에서 시작할건지를 의미한다.

* MAINTAINER : 이미지를 생성한 개발자의 정보 (1.13.0 이후 사용 X)

* LABEL : 이미지에 메타데이터를 추가 (key-value 형태)

* RUN : 새로운 레이어에서 명령어를 실행하고, 새로운 이미지를 생성한다.
> RUN 명령을 실행할 때 마다 레이어가 생성되고 캐시된다. 따라서 RUN 명령을 따로 실행하면 apt-get update는 다시 실행되지 않아서 최신 패키지를 설치할 수 없다. RUN 명령 하나에 apt-get update와 install을 함께 실행 해야한다.

* WORKDIR : 작업 디렉토리를 지정한다. 해당 디렉토리가 없으면 새로 생성한다.
> 작업 디렉토리를 지정하면 그 이후 명령어는 해당 디렉토리를 기준으로 동작한다. cd 명령어와 동일하다.

* EXPOSE : Dockerfile의 빌드로 생성된 이미지에서 열어줄 포트를 의미한다.
> 호스트 머신과 컨테이너의 포트 매핑시에 사용된다. 컨테이너 생성 시 -p 옵션의 컨테이너 포트 값으로 EXPOSE 값을 적어야한다.

* USER : 이미지를 어떤 계정에서 실행 하는지 지정
> 기본적으로 root에서 해준다.

* COPY / ADD : build 명령 중간에 호스트의 파일 또는 폴더를 이미지에 가져오는 것
> ADD 명령문은 좀 더 파워풀한 COPY 명령문이라고 생각할 수 있다. ADD 명령문은 일반 파일 뿐만 아니라 압축 파일이나 네트워크 상의 파일도 사용할 수 있다. 이렇게 특수한 파일을 다루는 게 아니라면 COPY 명령문을 사용하는 것이 권장된다.

* ENV : 이미지에서 사용할 환경 변수 값을 지정한다.
> path 등

* CMD / ENTRYPOINT : 컨테이너를 생성,실행 할 때 실행할 명령어
> docker run 명령으로 컨테이너를 생성하거나, docker start 명령으로 정지된 컨테이너를 시작할 때 실행된다. 보통 컨테이너 내부에서 항상 돌아가야하는 서버를 띄울 때 사용한다.

* CMD : CMD는 docker run 실행 시, 추가적인 명령어에 따라 설정한 명령어를 수정하고자 할 때 사용된다.

    * CMD 명령은 3가지 형태가 있다.
    ```bash
    CMD [“executable”,”param1”,”param2”]
    CMD [“param1”,”param2”]
    CMD command param1 param2
    ```

* ENTRYPOINT : docker run 실행 시, 추가적인 명령어의 존재 여부와 상관 없이 무조건 실행되는 명령이다.

    * ENTRYPOINT 명령은 2가지 형태가 있다.
    ```bash
    ENTRYPOINT [“executable”, “param1”, “param2”]
    ENTRYPOINT command param1 param2
    ```

* ARG : Argument 설정


다음은 ROS의 Dockerfile 

```bash
ARG FROM_IMAGE=ros:foxy
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
RUN echo "\
repositories: \n\
  ros2/demos: \n\
    type: git \n\
    url: https://github.com/ros2/demos.git \n\
    version: ${ROS_DISTRO} \n\
" > ../overlay.repos
RUN vcs import ./ < ../overlay.repos

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths \
        src/ros2/demos/demo_nodes_cpp \
        src/ros2/demos/demo_nodes_py \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --packages-select \
        demo_nodes_cpp \
        demo_nodes_py \
      --mixin $OVERLAY_MIXINS

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# run launch file
CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
```

### 생성한 Dockerfile을 Image로 빌드

```bash
docker build [OPTIONS] PATH | URL | -
```

* -t 옵션 : 생성할 이미지 이름 지정