---
sort: 4
---

# Package.xml

## 0. 참고 문헌
*- [ROS 2.0 package creating tutorials](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html)*

*- [REP 149](https://www.ros.org/reps/rep-0149.html)*


## 1. package.xml
`package.xml` 파일은 패키지의 정보를 담은 `XML` 파일이다. 패키지의 이름, 저작자, 라이선스, 의존성 패키지 등의 정보를 담고 있다.

## 2. package.xml 포맷
`ROS 2.0`에서 포맷 3을 사용한다. 다음은 포맷 3의 기본적인 형태이다.

```xml
<?xml version="1.0"?>
<?xml-model
  href="http://download.ros.org/schema/package_format3.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
<name>my_package</name>
<version>0.0.0</version>
<description>TODO: Package description</description>
<maintainer email="user@todo.todo">user</maintainer>
<license>TODO: License declaration</license>

<buildtool_depend>ament_cmake</buildtool_depend>

<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>

<export>
  <build_type>ament_cmake</build_type>
</export>
</package>
```

## 3. 패키지 정보
* `<?xm version="1.0?>` : `xml` 버전을 명시. 보통 `1.0`을 사용한다.
* `<package format="3">`: `format 3`형태의 `ROS` 패키지 설정 부분
* `<name>` : 패키지 이름 명시
* `<version>` : 패키지의 버전. 패키지를 release 하게 된다면, 버전 관리도 중요하다.
* `<description>` : 패키지에 대한 간략한 설명
* `<maintainer>` : 패키지 관리자의 이름과 이메일 주소 기재
* `<license>` : 패키지의 라이센스 기재.(BSD, MIT, Apache, ...)
* `<url>` : 패키지와 관련된 url 기재.(깃북, 저장소, 웹페이지, ...)
* `<author>` : 패키지 개발에 참여한 개발자들의 이름과 이메일 주소 기재

## 4. Dependency
패키지의 의존성과 관련된 사항을 명시하는 부분이다.

* `<buildtool_depend>` : 빌드 시스템의 의존성 기술. `ROS 2.0`은 `ament_cmake` 빌드 시스템을 사용하므로, `ament_cmake` 입력
* `<depend>` : 반복을 피하기위해 사용되며, `<build_depend>`, `<build_export_depend>`, `<exec_depend>`의 기능을 모두 합친 명령으로, 반복적으로 같은 패키지를 명시하는 것을 피하기 위해 사용된다.
* `<build_depend>` : 패키지 빌드 시 필요한 의존성 패키지 추가
* `<build_export_depend>` : 현재 패키지의 의존성 패키지가 다른 패키지의 빌드시 필요한 의존성 패키지인 경우 추가
* `<exec_depend>` : 패키지 실행 시 필요한 의존성 패키지 추가
* `<test_depend>` : 패키지 테스트 시 필요한 의존성 패키지 추가
* `<doc_depend>` : 문서 파일을 빌드하는데 필요한 의존성 패키지 추가
* `<conflict>` : 패키지와 충돌하는 패키지 추가
* `<replace>` : 현재 패키지로 대체할 수 있는 패키지 명시

## 5. export
* `<export>` : ROS에서 명시하지 않은 태그명을 사용할 때 사용, 다양한 패키지 및 서브시스템의 추가 정보를 명시한다. 이 때, 태그의 이름이 해당 태그를 처리하는 패키지와 동일해야 한다. 특히, ROS 2.0 에서는 `<build_type>`을 항상 명시한다.

```xml
# build_type
<export>
  <build_type>ament_cmake</build_type>
</export>

# rviz 태그의 plugin 명시
<export>
  <rviz plugin="${prefix}/plugin_description.xml"/>
</export>

# gazebo_ros 태그의 모델 경로 명시
<export>
  <gazebo_ros gazebo_model_path="${prefix}/models"/>
</export>
```