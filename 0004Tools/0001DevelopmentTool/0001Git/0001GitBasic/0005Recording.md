---
sort: 5
---

# Recording Changes

## 0. 참고 문헌

*- [Pro Git](https://git-scm.com/book/ko/v2)*

*- [Pro Git-수정하고 저장소에 저장하기](https://git-scm.com/book/ko/v2/Git%EC%9D%98-%EA%B8%B0%EC%B4%88-%EC%88%98%EC%A0%95%ED%95%98%EA%B3%A0-%EC%A0%80%EC%9E%A5%EC%86%8C%EC%97%90-%EC%A0%80%EC%9E%A5%ED%95%98%EA%B8%B0)*

## 1. 파일의 라이프사이클

다음은 Git에서 관리되는 파일의 라이프사이클이다.

<img src="lifecycle.png"  width="900" height="350">

* Tracked : 이미 관리되고 있던 파일
    * Unmodified : 수정되지 않은 파일
    * Modified : 수정된 파일
    * Staged : 커밋하면 저장소에 기록될 파일
* Untracked : 그 이외의 파일

이에 따른 라이프사이클은 다음과 같다.
1. 처음 저장소를 Clone 하면 모든 파일은 Tracked이면서 Unmodified 상태
2. 마지막 커밋 이후 어떤 파일을 수정하면 Modified 상태
3. 커밋하기 위해 수정한 파일을 Staged 상태로 만듦
4. Staged 상태의 파일을 커밋

## 2. status
이런 파일의 상태를 확인하기 위한 명령어가 `status`이다.
```bash
git status
```

또한 다음의 명령으로 상태 정보를 짧게 확인할 수 있다.
```bash
git status -s
git status --short
```

그 결과는 다음과 같으며, `??`는 Untracked 파일, `A`는 Staged 상태로 추가한 파일 중 새로 생성한 파일, `M`은 Modified된 경우를 말한다. 또한 상태정보는 두 개의 칼럼으로 구성되는데, 첫 번째 칼럼은 Staging Area의 상태를, 두 번째 칼럼은 Working Tree에서의 상태를 표시한다.

```bash
 M README
MM Rakefile
A  lib/git.rb
M  lib/simplegit.rb
?? LICENSE.txt
```

`README` 파일 같은 경우 내용을 변경했지만 아직 Staged 상태로 추가하지는 않았다. `lib/simplegit.rb` 파일은 내용을 변경하고 Staged 상태로 추가까지 한 상태이다. 위 결과에서 차이점을 비교해보자. `Rakefile` 은 변경하고 Staged 상태로 추가한 후 또 내용을 변경해서 Staged 이면서 Unstaged 상태인 파일이다.

## 3. 라이프 사이클의 예
### 3.1 untracked -> staged -> commit
git clone을 이용하여 repo에 새로운 파일을 만들면 그 파일은 Untracked files로 분류된다.

이 파일을 git add 명령을 통해 해당 파일을 tracked files로 분류시키며, 스테이징 파일이 된 것을 알 수 있다.

```bash
git add <file-name>
```

이 stage된 파일을 commit 명령을 통해 새로운 버전으로 커밋을 할 수 있다.

```bash
git commit <file-name>
```

### 3.2 modified -> staged -> commit
이번에는 git clone을 이용하여 새로운 파일을 만드는 것이 아닌, repo의 기존 파일을 수정하면 이 파일은 원래 추적되던 파일이므로, Untracked 파일이 아닌 Modified된 파일로 분류된다. 

이 파일을 위와 같이 add 및 commit의 단계를 거치면 새로운 버전으로 커밋이 된다.

