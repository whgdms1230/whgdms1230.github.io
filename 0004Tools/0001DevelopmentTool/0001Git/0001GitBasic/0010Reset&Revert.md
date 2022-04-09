---
sort: 10
---

# Reset&Revert

## 0. 참고 문헌

*- [Pro Git](https://git-scm.com/book/ko/v2)*

*- [Pro Git-되돌리기](https://git-scm.com/book/ko/v2/Git%EC%9D%98-%EA%B8%B0%EC%B4%88-%EB%90%98%EB%8F%8C%EB%A6%AC%EA%B8%B0)*

*- [Pro Git-Reset 명확히 알고 가기](https://git-scm.com/book/ko/v2/Git-%EB%8F%84%EA%B5%AC-Reset-%EB%AA%85%ED%99%95%ED%9E%88-%EC%95%8C%EA%B3%A0-%EA%B0%80%EA%B8%B0)*

*- [Git Revert](https://git-scm.com/docs/git-revert)*

## 1. 커밋 재작성

`commit --amend` 옵션을 이용하여 커밋 내용을 재작성 할 수 있다.

```bash
git commit --amend
```
## 2. Reset

`reset` 명령은 현재 브랜치의 커밋 내용을 이전의 내용으로 되돌리거나, 인덱스나 워킹 디렉토리에 있는 파일의 상태를 이전 상태로 되돌릴 수 있다.

### 2.1 이전 커밋으로 reset

현재 커밋에서 이전의 특정 커밋으로 reset시키는 명령으로, 이동하고자 하는 커밋 이후에 작성된 커밋 내용들은 모두 삭제되며, 옵션에 따라 해당 커밋 이후의 변경 사항들은 staged, unstaged 상태이거나 모두 삭제될 수 있다.

#### 2.1.1 reset --soft

`reset --soft` 명령은 현재의 커밋에서 특정 커밋으로 reset할 때, 되돌아가고자 하는 커밋 이후에 변경된 모든 내용들은 모두 staged 상태로 reset된다.

```bash
# 부모 commit으로 reset
git reset --soft HEAD~

# 특정 commit ID로 reset
git reset --soft <commit ID>
```

#### 2.1.2 reset --mixed

`reset --mixed` 명령은 현재의 커밋에서 특정 커밋으로 reset할 때, 되돌아가고자 하는 커밋 이후에 변경된 모든 내용들은 모두 unstaged 상태로 reset된다.

```bash
# 부모 commit으로 reset
git reset --mixed HEAD~

# 특정 commit ID로 reset
git reset --mixed <commit ID>
```

#### 2.1.3 reset --hard

`reset --mixed` 명령은 현재의 커밋에서 특정 커밋으로 reset할 때, 되돌아가고자 하는 커밋 이후에 변경된 모든 내용들은 모두 삭제된다.

```bash
# 부모 commit으로 reset
git reset --hard HEAD~

# 특정 commit ID로 reset
git reset --hard <commit ID>
```

### 2.2 파일 상태 변화

reset 명령을 이용하여 파일의 상태를 staged 파일에서 unstaged 파일로 변경하거나, modified 파일을 이전 커밋의 파일 상태로 되돌릴 수 있다.

```bash
git reset HEAD <file>
```

위와 같이, file에 상태를 변경하고자 하는 파일을 작성하면, 해당 file의 상태를 변경할 수 있다. 만약 staged 파일이었다면, unstaged 상태로 바뀌고, unstaged 상태인 파일이 modified 상태였다면, 이전 커밋의 파일 상태로 되돌아가게 된다.

## 3. Revert

`revert` 명령어는 특정 커밋 상태로 되돌림과 동시에 되돌리는 새로운 커밋을 생성한다. 즉, `reset` 명령과 같이 HEAD를 이전에 생성한 특정 커밋으로 되돌리는 것이 아닌, 새로운 커밋을 생성하면서 특정 커밋 상태로 돌리기 때문에 원격저장소와 충돌이 발생하지 않는다.

다음과 같이 특정 commit ID에 대하여 `revert`를 하면 해당 커밋으로 내용을 되돌리며, 커밋이 자동으로 새로 생성된다.

```bash
git revert <commit ID>
git revert <commit ID 1> <commit ID 2> ...  # 여러 개 commit revert
git revert HEAD       # 바로 직전 커밋 revert
git revert HEAD~N..   # 현재 커밋에서 N개 만큼 revert
```

만약 revert한 결과를 stage 상태로 만들기만 하고 commit은 하지 않으려면 `--no-commit` 옵션을 추가한다.

```cpp
git revert <commit ID> --no-commit
```

`revert`를 사용하는 경우는 협업 시 커밋을 특정 커밋으로 되돌리고자 할 때 주로 발생한다. 특정 커밋으로 되돌리기에 `reset`이라는 명령어는 강력하지만, `reset`을 사용하면 원격 저장소에 push할 때 강제로 push 해야하는데, 이는 협업 시에 문제가 될 수 있다. 반면 `revet`는 이전 커밋으로 되돌리는 이력이 커밋으로 남기 때문에, 해당 커밋을 원격 저장소에 push하는데 아무런 문제가 되지 않는다.

`revert`로 커밋을 되돌리고자 할 때, 되돌아가고자 하는 커밋의 역순으로 `revert` 해야 한다. 만약 그렇게 하지 않으면 충돌이 발생할 수 있다.
예를 들어 Commit 1 -> Commit 2 -> Commit 3 순으로 커밋 히스토리가 쌓인 상태에서, 현재 Commit 3에서 Commit 1으로 되돌아가고자 할 때, Commit 3 -> Commit 2 순서대로 `revert` 명령을 내려야 한다.

만약 돌아가야 하는 커밋의 갯수가 많다면 `--no-commit` 옵션을 이용하여 `revert`를 위한 커밋을 한 개만 생성할 수 있다.