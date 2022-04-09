---
sort: 14
---

# Merge

## 0. 참고 문헌

*- [Pro Git](https://git-scm.com/book/ko/v2)*

*- [Pro Git-브랜치와 Merge의 기초](https://git-scm.com/book/ko/v2/Git-%EB%B8%8C%EB%9E%9C%EC%B9%98-%EB%B8%8C%EB%9E%9C%EC%B9%98%EC%99%80-Merge-%EC%9D%98-%EA%B8%B0%EC%B4%88)*

*- [Pro Git-고급 Merge](https://git-scm.com/book/ko/v2/Git-%EB%8F%84%EA%B5%AC-%EA%B3%A0%EA%B8%89-Merge)*

## 1. Merge란?

Merge는 분기(base brach)가 같은 브랜치 간의 내용을 병합하는 과정이다.

다음의 상황은 `C2` 커밋에 `master` 브랜치가 있고, `master` 브랜치에서 `iss53` 브랜치와 `hotfix` 브랜치를 만들어 각각 `C3` 커밋과 `C4` 커밋을 추가한 상황이다. 즉, `master` 브랜치의 `C2` 커밋에서 `hotfix` 브랜치와 `iss53` 브랜치 두 개가 파생된 것이다.

<img src="merge1.png"  width="800" height="400">

`hotfix` 브랜치에서 작업을 완료하여 `master` 브랜치에 `hotfix` 브랜치를 `merge` 시킨다. 이 때는 `master` 브랜치가 `hotfix` 브랜치의 베이스 브랜치이기 때문에 충돌이 발생하지 않는다. 따라서 `master` 브랜치도 `merge` 함으로써 `C4` 브랜치를 가리키게 된다.

```bash
git checkout master # master 브랜치에 머지하기 위해 master 브랜치로 체크아웃
git merge hotfix # hotfix 브랜치를 현재 브랜치인 master 브랜치에 머지
```

<img src="merge2.png"  width="800" height="500">

`hotfix` 브랜치는 작업이 끝나고 더이상 필요하지 않으므로 삭제하고, `iss53` 브랜치는 추가 작업을 진행하여 `C5` 커밋을 생성한 상태이다.

<img src="merge3.png"  width="800" height="400">

`iss53` 브랜치를 `master` 브랜치로 머지하려고 하나, 공통된 베이스 브랜치가 존재하지만 수정된 내용이 충돌될 경우 바로 머지되지 않는다.

```bash
git checkout master
git merge iss53
```

<img src="merge4.png"  width="800" height="400">

충돌이 발생하면 해당 충돌을 해결 한 이후에 머지가 된다. git은 `fast-forward`로 머지하지 않고, 해당 커밋 두 개와 공통 조상 커밋 하나를 사용하여 `3-way Merge`를 한다. 머지가 완료되면 머지에 대한 `C6` 커밋이 생성되고 `master` 브랜치는 해당 커밋을 가리킨다.

<img src="merge5.png"  width="800" height="400">

## 2. 충돌

`3-way Merge`가 실패하는 경우도 발생한다. 머지하고자 하는 두 브랜치에서 같은 파일의 한 부분을 동시에 수정하면 git에서 머지를 하지 못하고 `conflict`를 발생시킨다.

충돌이 발생하면, 충돌을 직접 해결해주지 않는 한 머지를 진행할 수 없다. `git status` 명령을 이용하면 다음과 같은 충돌 메시지를 보여준다. 충돌이 일어난 파일은 `unmerged` 상태로 표시된다.

```bash
On branch master
You have unmerged paths.
  (fix conflicts and run "git commit")

Unmerged paths:
  (use "git add <file>..." to mark resolution)

    both modified:      index.html

no changes added to commit (use "git add" and/or "git commit -a")
```

충돌된 부분은 다음과 같이 표시해주는데, 개발자가 해당 내용을 보고 어떤 내용으로 머지를 수행할 지 해결해주어야 한다.
```bash
 <<<<<<< HEAD:index.html
 <div id="footer">contact : email.support@github.com</div>
 =======
 <div id="footer">
  please contact us at support@github.com
 </div>
 >>>>>>> iss53:index.html
```

해당 부분을 해결하고 다시 `merge` 작업을 수행하면 된다.

> 그 이외에 merge 도구를 이용하여 충돌을 해결할 수 있는데, `git mergetool` 명령으로 실행할 수 있다.

## 3. Merge 취소하기

`git merge --abort` 명령으로 머지하기 전 상태로 되돌릴 수 있다. 또는 merge를 처음부터 다시하고 싶다면 `git reset --hard HEAD` 명령으로 되돌릴 수 있다. 단, 이 명령은 워킹디렉토리를 그 시점으로 완전히 되돌려서 저장하지 않은 것까지 사라진다는 점을 유의해야 한다.

## 4. Merge 되돌리기

다음과 같은 상황일 때를 가정한다.

<img src="merge6.png"  width="800" height="400">

커밋을 되돌리는 데는 두 가지 방법이 있으며, 두 방법은 결과가 다르다.

### 4.1 Refs 수정

실수로 생긴 Merge 커밋이 로컬 저장소에만 있을 때는 `git reset --hard HEAD~` 명령으로 브랜치를 되돌리면 된다. 이 방법은 원격 저장소에 push된 상황에서 `reset` 하게 되면 문제가 발생할 수 있다.

<img src="merge7.png"  width="800" height="400">

### 4.2 커밋 되돌리기

`revert`를 이용하여 모든 변경사항을 취소하는 새로운 커밋을 만들 수 있다.

```bash
git revert -m 1 HEAD
```

`-m 1` 옵션은 부모가 보호되어야하는 mainline 이라는 것을 나타낸다. `HEAD`로 `merge`를 했을 때 `merge` 커밋은 두 개의 부모 커밋을 가진다. 첫 번째 부모 커밋은 `HEAD(C6)`이고 두 번째 부모 커밋은 `merge` 대상 브랜치 `C4`이다. `C4`에서 받아온 모든 변경사항을 되돌리고 `C6`로부터 받아온 변경사항은 남겨두고자 하는 상황이다.

다음은 `revert`를 한 이후의 히스토리이다. `revert`를 통해 새로 생성된 커밋 `^M`은 `C6`와 내용이 똑같다. 이는 `merge`하기 이전 커밋인 `C6`와 완전히 같은 내용이므로 `merge` 이전으로 되돌아간 것이다. 

<img src="merge8.png"  width="800" height="400">

만약에 이 상태에서 다시 `merge`를 한다고 하면 다음과 같은 메시지가 출력된다. 이미 merge 됐던 `topic` 브랜치는 더 이상 `master` 브랜치로 머지될 내용이 없다. 이미 히스토리에 머지된 기록이 남아있기 때문이다.

```bash
Already up-to-date
```