---
sort: 8
---

# Add & Commit

## 0. 참고 문헌

*- [Pro Git](https://git-scm.com/book/ko/v2)*

*- [Pro Git-수정하고 저장소에 저장하기](https://git-scm.com/book/ko/v2/Git%EC%9D%98-%EA%B8%B0%EC%B4%88-%EC%88%98%EC%A0%95%ED%95%98%EA%B3%A0-%EC%A0%80%EC%9E%A5%EC%86%8C%EC%97%90-%EC%A0%80%EC%9E%A5%ED%95%98%EA%B8%B0)*

## 1. Add

git add는 working tree 상의 변경 내용을 스테이지 영역에 추가하는 명령어이다. 즉 Untracked 파일이나, Modified 파일을 Stage 영역에 추가함으로써, Commit 명령 시 새로운 버전에 등록되는 파일을 추가하는 것이다.

다음은 특정 파일을 staging 하는 명령이다.

```bash
git add [file name]
```

현재 디렉토리의 모든 변경 내용을 staging 하는 경우에 다음과 같이 명령한다.

```bash
git add .
```

만약 작업 디렉토리 내의 모든 변경내용을 staging 하는 경우 다음과 같이 명령한다.

```bash
git add -A
```

또한 각 변경사항을 하나하나 확인하여 스테이징 하는 경우에는 다음과 같은 옵션을 사용한다.

```bash
git add -p
```

## 2. commit

커밋하기 위해서는 Unstaged 상태의 파일들을 git add 명령으로 추가해야 한다. 만약 생성하거나 수정하고 나서 git add 명령으로 추가하지 않은 파일은 커밋되지 않는다. 따라서 커밋하기 전에 git status 명령으로 모든 것이 Staged 상태인지 확인하여 원하는 파일이 커밋될 수 있는 상태인지 확인하고, 그 후에 git commit 을 실행하여 커밋하는 것이 좋다.

```bash
git commit
```

해당 명령을 실행하면, Git 설정에 지정된 편집기가 실행되고, 아래와 같은 텍스트가 자동으로 포함된다.

> 아래 예제는 Vim 편집기의 화면이다. 이 편집기는 쉘의 EDITOR 환경 변수에 등록된 편집기이고 보통은 Vim이나 Emacs을 사용한다. 또 시작하기 에서 설명했듯이 git config --global core.editor 명령으로 어떤 편집기를 사용할지 설정할 수 있다.

```bash
# Please enter the commit message for your changes. Lines starting
# with '#' will be ignored, and an empty message aborts the commit.
# On branch master
# Your branch is up-to-date with 'origin/master'.
#
# Changes to be committed:
#	new file:   README
#	modified:   CONTRIBUTING.md
#
~
~
~
".git/COMMIT_EDITMSG" 9L, 283C
```

git commit 에 -v 옵션을 추가하면 편집기에 diff 메시지도 추가된다

```bash
git commit -v
```

commit 명령을 실행할 때 아래와 같이 -m 옵션을 사용하면 커밋 메시지를 인라인으로 작성할 수 있다.

```bash
git commit -m "Story 182: Fix benchmarks for speed"
```

git commit 명령을 실행할 때 -a 옵션을 추가하면 Tracked 상태의 파일을 자동으로 Staging Area에 넣는데, 이는git add 명령을 실행하는 수고를 덜 수 있다.

```bash
git commit -a
```

## 3. Commit Messages

