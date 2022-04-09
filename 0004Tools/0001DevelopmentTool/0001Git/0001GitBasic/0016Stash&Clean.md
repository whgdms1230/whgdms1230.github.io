---
sort: 16
---

# Stash & Clean

## 0. 참고 문헌

*- [Pro Git-Stashing과 Cleaning](https://git-scm.com/book/ko/v2/Git-%EB%8F%84%EA%B5%AC-Stashing%EA%B3%BC-Cleaning)*

## 1. Stash

작업하던 일이 있고 다른 요청이 들어와서 잠시 브랜치를 변경해야 할 일이 생겼다고 치자. 그런데 이런 상황에서 아직 완료하지 않은 일을 커밋하는 것이 껄끄럽다는 것이 문제다. 커밋하지 않고 나중에 다시 돌아와서 작업을 다시 하고 싶을 것이다. 이 문제는 `git stash` 라는 명령으로 해결할 수 있다.

Stash 명령을 사용하면 워킹 디렉토리에서 수정한 파일들만 저장한다. Stash는 Modified이면서 Tracked 상태인 파일과 Staging Area에 있는 파일들을 보관해두는 장소다. 아직 끝내지 않은 수정사항을 스택에 잠시 저장했다가 나중에 다시 적용할 수 있다(브랜치가 달라져도 말이다).

만약 작업 중인 브랜치에서 두 개의 파일을 수정하고, 하나의 파일은 Staging Area에 Add 한 상황에서 `git status` 명령을 하면 다음과 같은 상태 정보가 올라오낟.

```bash
Changes to be committed:
  (use "git reset HEAD <file>..." to unstage)

  modified:   index.html

Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git checkout -- <file>..." to discard changes in working directory)

  modified:   lib/simplegit.rb
```

작업 중 다른 브랜치에서 작업을 하기 위해 checkout을 하려고 하지만, 작업하고 있던 내용이 있어 checkout을 하지 못한다. 하지만 작업 중인 파일들은 아직 커밋할 단계가 아니다. 이런 경우에 stash를 하면 워킹 디렉토리를 비우고 브랜치를 변경할 수 있다.

```bash
git stash
```

`git status`로 확인하면 워킹 디렉토리가 비워진 것을 확인할 수 있다.

```bash
# On branch master
nothing to commit, working directory clean
```

stash로 작업하던 내용은 지워진 것이 아니라, 스택에 저장된 것이다. 다음의 명령으로 stash된 리스트들을 확인할 수 있다.

### list 옵션

```bash
git stash list
```

리스트를 확인했을 때 다음과 같은 리스트 정보가 확인된다. 현재 워크스페이스에 stash된 내용은 총 3 건이 스택에 저장되어 있음을 알 수 있다.

```bash
stash@{0}: WIP on master: 049d078 added the index file
stash@{1}: WIP on master: c264051 Revert "added file_size"
stash@{2}: WIP on master: 21d80a5 added number to log
```

### apply 옵션

stash된 내용을 다시 불러오려면 `apply` 옵션을 사용하면 된다. 이 명령은 가장 최근에 stash한 내용을 다시 불러온다. 즉 위에서 stash한 `stash@{0}` 내용을 가져온다.

```bash
git stash apply
```

> 꼭 깨끗한 워킹 디렉토리나 Stash 할 때와 같은 브랜치에 적용해야 하는 것은 아니다. 어떤 브랜치에서 Stash 하고 다른 브랜치로 옮기고서 거기에 Stash를 복원할 수 있다. 그리고 꼭 워킹 디렉토리가 깨끗한 상태일 필요도 없다. 워킹 디렉토리에 수정하고 커밋하지 않은 파일들이 있을 때도 Stash를 적용할 수 있다. 만약 충돌이 있으면 알려준다.

Git은 Stash를 적용할 때 Staged 상태였던 파일을 자동으로 다시 Staged 상태로 만들어 주지 않는다. 그래서 git stash apply 명령을 실행할 때 `--index` 옵션을 주어 Staged 상태까지 적용한다. 그래야 원래 작업하던 상태로 돌아올 수 있다.

```bash
git stash apply --index
```

`apply` 옵션은 단순히 stash를 불러오기만 한다. 따라서 불러온 stash 내용을 스택에서 지우려면 `drop` 옵션을 사용해야 한다. 이 명령은 가장 최근에 stash한 내용을 drop 한다.

### drop 옵션

```bash
git stash drop
```

`apply` 옵션과 `drop` 옵션을 한 번에 수행하는 옵션은 `pop` 옵션이다. 이 옵션을 이용하면 가장 최근의 stash를 가져오면서 스택에서 해당 stash를 제거해준다.

### pop 옵션

```bash
git stash pop
```

### stash 이름 명시하기

가장 최근의 stash가 아닌 그 이전의 stash 내용을 불러오거나 지울 경우에는 stash 이름을 입력하면 골라서 적용할 수 있다.

```bash
git stash apply stash@{2}
git stash drop stash@{2}
git stash pop stash@{2}
```

### --keep-index 옵션

`--keep-index` 옵션을 사용하면 Staging Area에 등록된 파일을 stash 하지 않는다.

```bash
git stash --keep-index
```

### --include-untracked, -u 옵션

기본적으로 stash 는 추적 중인 파일만 저장한다. 추적 중이지 않은 파일을 같이 저장하려면 Stash 명령을 사용할 때 `--include-untracked`나 `-u` 옵션을 붙여준다.

```bash
git stash -u
git stash --include-untracked
```

### --patch 옵션

`--patch` 옵션을 붙이면 Git은 수정된 모든 사항을 저장하지 않는다. 대신 대화형 프롬프트가 뜨며 변경된 데이터 중 저장할 것과 저장하지 않을 것을 지정할 수 있다.

```bash
git stach --patch
```

## 2. Stash를 적용한 브랜치 만들기

수정한 파일에 Stash를 적용하면 충돌이 일어날 수도 있고 그러면 또 충돌을 해결해야 한다. 필요한 것은 Stash 한 것을 쉽게 다시 테스트하는 것이다. `git stash branch <Branch Name>` 명령을 실행하면 Stash 할 당시의 커밋을 Checkout 한 후 새로운 브랜치를 만들고 여기에 적용한다. 이 모든 것이 성공하면 Stash를 삭제한다.

```bash
git stash branch <Branch Name>
```

## 3. Clean

작업하고 있던 파일을 Stash 하지 않고 단순히 그 파일들을 치워버리고 싶을 때가 있다. `git clean` 명령이 그 일을 한다.

```bash
git clean
```

보통은 Merge나 외부 도구가 만들어낸 파일을 지우거나 이전 빌드 작업으로 생성된 각종 파일을 지우는 데 필요하다.

이 명령을 사용할 때는 신중해야 한다. 이 명령을 사용하면 워킹 디렉토리 안의 추적하고 있지 않은 모든 파일이 지워지기 때문이다.

추적 중이지 않은 모든 정보를 워킹 디렉토리에서 지우는 경우에는 `git clean -f -d` 명령을 사용한다. 이 명령은 하위 디렉토리까지 모두 지워버린다.

```bash
git clean -f -d
```

이 명령을 실행했을 때 어떤 일이 일어날지 미리 보고 싶다면 `-n` 옵션을 사용한다. `-n` 옵션은 "가상으로 실행해보고 어떤 파일들이 지워질지 알려달라" 라는 뜻이다.

```bash
git clean -n
```