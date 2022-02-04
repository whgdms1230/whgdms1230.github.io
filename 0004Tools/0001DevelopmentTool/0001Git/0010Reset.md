---
sort: 10
---

# Reset

## 0. 참고 문헌

*- [Pro Git](https://git-scm.com/book/ko/v2)*

*- [Pro Git-되돌리기](https://git-scm.com/book/ko/v2/Git%EC%9D%98-%EA%B8%B0%EC%B4%88-%EB%90%98%EB%8F%8C%EB%A6%AC%EA%B8%B0)*

*- [Pro Git-Reset 명확히 알고 가기](https://git-scm.com/book/ko/v2/Git-%EB%8F%84%EA%B5%AC-Reset-%EB%AA%85%ED%99%95%ED%9E%88-%EC%95%8C%EA%B3%A0-%EA%B0%80%EA%B8%B0)*

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