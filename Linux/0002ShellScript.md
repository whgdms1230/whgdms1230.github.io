---
sort: 2
---

# ShellScript&bashrc

## 0. 참고 문헌
*- [Shell Programming Tutorial](https://wiki.kldp.org/wiki.php/ShellProgrammingTutorial)*

*- [Shell Scrpti Wiki Pages](https://ko.wikipedia.org/wiki/%EC%85%B8_%EC%8A%A4%ED%81%AC%EB%A6%BD%ED%8A%B8)*

*- [[dohk::인공지능과 생명정보학] 쉘의 개념, bashrc의 개념](https://dohk.tistory.com/191)*

## 1. Shell Script

`shell`이란 운영체제에서 **사용자가 입력하는 명령을 읽고 해석하여 대신 실행해주는 프로그램**을 말한다. 운영체제 상에서 다양한 운영체제 기능과 서비스를 구현하는 인터페이스를 제공하며, 사용자와 운영체제의 내부(커널) 사이의 인터페이스를 감싸는 층이기 때문에 셸이라는 이름이 붙었다.

`shell script`는 셸이나 명령 줄 인터프리터에서 돌아가도록 작성되었거나 한 운영 체제를 위해 쓰인 스크립트로, 단순한 도메인 고유 언어로 여기기도 한다. 셸 스크립트가 수행하는 일반 기능으로는 파일 이용, 프로그램 실행, 문자열 출력 등이 있음. `.sh`라는 파일 확장자를 가진 파일이 특정 종류의 셸 스크립트를 가리키는 것이 보통이지만, 대부분의 셸 스크립트는 파일 확장자를 지니지 않는다.

## 2. 문법

해당 절에서는 [Shell Programming Tutorial](https://wiki.kldp.org/wiki.php/ShellProgrammingTutorial)의 문법을 참조할 것.

## 3. bachrc

`bash`는 Bourne Again Shell의 축약어로, 리눅스에서 가장 널리 쓰이는 쉘이다. bash는 다음의 다섯 개의 공통된 설정 파일을 갖는다. 

* /etc/profile
* /etc/bashrc
* ~/.bash_profile
* ~/.bashrc
* ~/.bash_logout

/etc 디렉토리에 위치한 설정 파일은 전역적인 파일이다. 지역적인 파일은 개별 설정을 위한 파일로 그 파일을 사용하는 특정 사용자에게만 영향을 끼친다.

### 3.1 /etc/prfile
환경 변수와 bash가 수행될 때 실행되는 프로그램을 제어하는 전역적인 시스템 설정과 관련된 파일, 로그인 시 수행되는 전체 시스템 환경 설정 파일

### 3.2 /etc/bashrc
별칭(alias)과 bash가 수행될 때 실행되는 함수를 제어하는 전역적인 시스템 설정과 관련된 파일, 때때로 /etc/bashrc는 생략되기도 하며 그 내용은 /etc/profile에 함께 포함되기도 함

### 3.3 ~/.bash_profile
환경 변수와 bash가 수행될 때 실행되는 프로그램을 제어하는 지역적인 시스템 설정과 관련된 파일, 이 파일은 전역적인 설정 파일인 /etc/profile이 수행된 다음 바로 수행

### 3.4 ~/.bashrc
별칭(alias)과 bash가 수행될 때 실행되는 함수를 제어하는 지역적인 시스템 설정과 관련된 파일

### 3.5 ~/.bash_logout
사용자가 로그 아웃하기 바로 직전에 실행하는 프로그램에 관한 bash의 지역적인 시스템 설정과 관련된 파일