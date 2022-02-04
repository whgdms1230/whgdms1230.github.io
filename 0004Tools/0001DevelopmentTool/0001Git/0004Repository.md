---
sort: 4
---

# Make Git Repository

## 0. 참고 문헌

*- [Pro Git](https://git-scm.com/book/ko/v2)*

*- [Pro Git-Git 저장소 만들기](https://git-scm.com/book/ko/v2/Git%EC%9D%98-%EA%B8%B0%EC%B4%88-Git-%EC%A0%80%EC%9E%A5%EC%86%8C-%EB%A7%8C%EB%93%A4%EA%B8%B0)*

## 1. 저장소 만들기

주로 다음 두 가지 중 한 가지 방법으로 Git 저장소를 쓰기 시작한다.

1. 아직 버전관리를 하지 않는 로컬 디렉토리 하나를 선택해서 Git 저장소를 적용하는 방법
2. 다른 어딘가에서 Git 저장소를 Clone 하는 방법

## 2. 기존 디렉토리를 Git 저장소로 만들기

Git 저장소를 만들고 싶은 레포지토리 위치에서 다음의 명령을 실행하면, Git 저장소로 만들어진다. 해당 명령은 `.git`이라는 하위 디렉토리를 만들고, .`git` 디렉토리에는 저장소에 필요한 뼈대 파일(Skeleton)이 들어 있다. 

```bash
git init
```

## 2. 기존 저장소를 Clone 하기
다른 프로젝트에 참여하려거나(Contribute) Git 저장소를 복사하고 싶을 때 git clone 명령을 사용한다.

```bash
git clone <git-url>
```

이 명령은 해당 레포지토리 이름에 해당하는 디렉토리를 만들고 그 안에 `.git` 디렉토리를 만든다. 그리고 저장소의 데이터를 모두 가져와서 자동으로 가장 최신 버전을 Checkout 해 놓는다.
