---
sort: 6
---

# ROS 2 Package Structure

## 0. 참고 문헌
*- ROS 2로 시작하는 로봇 프로그래밍(표윤석, 임태훈)*

## 1. 기본 설치 폴더와 사용자 작업 폴더
### 1.1 기본 설치 폴더
ROS 2의 기본 폴더는 /opt 폴더 아래에 /ros 폴더가 생성되고, /ros 폴더 아래에 버전별로 폴더가 생성된다. 예를 들어, foxy 버전을 설치했다면 /opt/ros/foxy가 설치 되고, foxy 폴더 아래에는 기본 패키지들이 설치가 되어있다. 해당 폴더에는 apt-get으로 받은 ros-rosdistro 로 시작하는 패키지들을 포함하여 해당 버전의 기본 패키지들이 설치되어 있다.

폴더의 세부 내용은 다음과 같다.
* /bin : 실행 가능한 바이너리 파일
* /cmake : 빌드 설정 파일
* /include : 헤더 파일
* /lib : 라이브러리 파일
* /opt : 기타 의존 패키지
* /share : 패키지의 빌드, 환경설정 파일
* local_setup.* : 환경설정 파일
* setup.* : 환경설정 파일

### 1.2 사용자 작업 폴더
사용자 작업 폴더는 사용자가 원하는 곳에 생성을 할 수 있으며, 대부분 /home 폴더 아래에 workspace를 생성한다.

폴더의 세부 내용은 다음과 같다.
* /build : 빌드 설정 파일용 폴더
* /install : msg, srv, action 헤더 파일과 사용자 패키지 라이브러리, 실행 파일용 폴더
* /log : 빌드 로깅 파일용 폴더
* /src : 사용자 패키지용 폴더

또한 src 폴더 내부에 일반적으로 가지는 폴더 구조는 다음과 같다.
* /src : C/C++ 코드용 폴더
* /include : C/C++ 헤더 파일용 폴더
* /param : 파라미터 파일용 펄더
* /launch : launch에 사용되는 launch 파일용 폴더
* /패키지_이름의_폴더 : 파이썬 코드용 폴더
* /test : 테스트 코드 및 테스트 데이터용 폴더
* /msg : 메시지 파일용 폴더
* /srv : 서비스 파일용 폴더
* /action : 액션 파일용 폴더
* /doc : 문서용 폴더
* package.xml : 패키지 설정 파일
* CMakeLists.txt : C/C++ 빌드 설정 파일
* setup.py : 파이썬 코드 환경 설정 파일
* README.md : 사용자 문서, github 레포 메인에 표시되는 문서
* CONTRIBUTING.md : 해당 패키지 개발에 공헌하는 방법을 기술하는 파일
* LICENSE : 이 패키지의 라이선스를 기술하는 파일
* CHANGELOG.rst : 이 패키지의 버전별 변경사항 모음 파일

## 2. 패키지 생성
ROS 2 패키지 생성 명령어는 다음과 같다.

```bash
ros2 pkg create [PACKAGE_NAME] --build-type [BUILD_TYPE] --dependencies [DEPENDNECY_PACKAGE_1] [DEPENDNECY_PACKAGE_N]
```

