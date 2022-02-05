---
sort: 3
---

# Config & Help

## 0. 참고 문헌

*- [Pro Git](https://git-scm.com/book/ko/v2)*

*- [Pro Git-Git 최초 설정](https://git-scm.com/book/ko/v2/%EC%8B%9C%EC%9E%91%ED%95%98%EA%B8%B0-Git-%EC%B5%9C%EC%B4%88-%EC%84%A4%EC%A0%95)*

*- [Pro Git-도움말 보기](https://git-scm.com/book/ko/v2/%EC%8B%9C%EC%9E%91%ED%95%98%EA%B8%B0-%EB%8F%84%EC%9B%80%EB%A7%90-%EB%B3%B4%EA%B8%B0)*

## 1. config

Git은 커밋할 때마다 사용자 이름과 이메일 주소 정보를 이용하여 커밋 정보를 저장한다. 따라서 사용자 이름과 이메일 주소 정보를 설정해야 하는데, 이 때 사용되는 명령어가 `config`이다.

```bash
git config --global user.name "PaulKim"
git config --global user.email whgdms1230@naver.com
```

`--global` 옵션은 해당 시스템에서 해당 사용자가 사용할 때는 이 정보를 사용한다. 만약 프로젝트마다 다른 이름과 이메일 주소를 사용하고 싶으면 `--global` 옵션을 빼고 명령을 실행한다.

```bash
git config user.name "PaulKim"
git config user.email whgdms1230@naver.com
```

설정된 설정 정보들을 확인할 때는 다음과 같이 전체 리스트를 확인 하거나, 두 번째 방법과 같이 특정 설정 값의 정보를 확인할 수 있다.

```bash
# 전체 설정 정보 확인
git config --list

# 특정 설정 정보 확인
git config user.name
```

그 이외에도 많은 설정값들이 있으며, 해당 설정값도 설정할 수 있다.

## 2. help

git의 명령어는 많기 때문에 help 명령을 통해 명령어의 도움말을 불러올 수 있다.

```bash
git help <verb>
```

또한 다음의 명령어를 이용하여 각 명령어에 대한 옵션을 확인할 수 있다.

```bash
git <verb> -h
git <verb> --help
```