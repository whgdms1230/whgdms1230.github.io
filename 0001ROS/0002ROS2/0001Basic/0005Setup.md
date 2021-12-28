---
sort: 5
---

# setup.py

## 0. 참고 문헌
*- [ROS 2.0 Package Documentation](https://docs.ros.org/en/foxy/How-To-Guides/Developing-a-ROS-2-Package.html)*

*- ROS 2로 시작하는 로봇 프로그래밍(표윤석, 임태훈)*

## 1. 파이썬 패키지 설정 파일 setup.py
`setup.py`는 ROS 2 파이썬 패키지에만 사용되는 파일로, 배포를 위한 설정 파일이다. 파이썬 패키지에서는 `CMakeLists.txt`는 사용하지 않지만, `package.xml` 파일은 ROS 패키지의 필수 구성요소이기 때문에 패키지에 포함시켜야 한다.

다음은 ROS 2.0 documentation 페이지에서 가져온 `setup.py` 예시이다.

```python
import os
from glob import glob
from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='ROS 2 Developer',
    author_email='ros2@ros.com',
    maintainer='ROS 2 Developer',
    maintainer_email='ros2@ros.com',
    keywords=['foo', 'bar'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='My awesome package.',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'my_script = my_package.my_script:main'
        ],
    },
)
```

* name : 패키지 이름
* version : 패키지의 버전
* packages : 의존하는 패키지
* data_files : 이 패키지에서 사용되는 파일들을 기입하여 함께 배포함
* install_requires : 의존하는 패키지로 이 패키지를 pip을 통해 설치할 때 함께 설치되는 패키지. ROS에서는 pip로 설치하지 않기 때문에 setuptools launch만을 기입한다.
* tests_require : 테스트에 필요한 패키지, ROS에서는 pytest를 사용한다.
* zip_safe : 설치 시 zip 파일로 아카이브할지 여부를 설정
* author, author_email, maintainer, maintainer_email : 저작자, 관리자의 이름과 이메일을 기입
* keywords : 이 패키지의 키워드
* classifiers : PyPI에 등록될 메타 데이터 설정
* description : 패키지 설명 기입
* license : 라이선스 종류 기입
* entry_points : 플랫폼 별로 콘솔 스크립트를 설치하도록 콘솔 스크립트 이름과 호출 함수를 기입

## 2. 파이썬 패키지 환경설정 파일 setup.cfg
`setup.cfg` 파일은 ROS 2 파이썬 패키지에서만 사용되며, `setup.py` 파일의 setup 함수에서 설정하지 못하는 기타 옵션을 `setup.cfg` 파일에 정의할 수 있다. [develop]과 [install]옵션을 설정하여 스크립트의 저장 위치를 설정하는데, 다음은 ROS 2.0 documentation 페이지에서 가져온 `setup.cfg` 예시이다.

```cfg
[develop]
script-dir=$base/lib/<package-name>
[install]
install-scripts=$base/lib/<package-name>
```