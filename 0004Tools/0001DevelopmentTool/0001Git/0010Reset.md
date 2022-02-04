---
sort: 10
---

# Reset

## 0. 참고 문헌

*- [Pro Git](https://git-scm.com/book/ko/v2)*

## 1. 커밋 재작성

`commit --amend` 옵션을 이용하여 커밋 내용을 재작성 할 수 있다.

```bash
git commit --amend
```
## 2. Reset

`reset` 명령은 현재 브랜치의 커밋 내용을 이전의 내용으로 되돌리거나, 인덱스나 워킹 디렉토리에 있는 파일의 상태를 이전 상태로 되돌릴 수 있다.

### 2.1 이전 커밋으로 reset

#### 2.1.2 reset --soft

`reset --soft` 명령은 현재의 커밋에서 특정 커밋으로 reset할 때, 되돌아가고자 하는 커밋 이후에 변경된 모든 내용들은 모두 staged 상태로 reset된다.

```bash
# 부모 commit으로 reset
git reset --soft HEAD~

# 특정 commit ID로 reset
git reset --soft <commit ID>
```

