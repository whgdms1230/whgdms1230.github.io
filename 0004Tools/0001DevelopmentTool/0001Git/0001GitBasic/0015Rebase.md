---
sort: 15
---

# Rebase

## 0. 참고 문헌

*- [Pro Git](https://git-scm.com/book/ko/v2)*

*- [Pro Git-Rebase 하기](https://git-scm.com/book/ko/v2/Git-%EB%B8%8C%EB%9E%9C%EC%B9%98-Rebase-%ED%95%98%EA%B8%B0)*

## 1. Rebase란?

Rebase란 한 브랜치에서 다른 브랜치로 합치는 또 다른 방법이다. 하지만 Rebase는 단순히 브랜치를 합치는 것이 아니라 하나의 브랜치의 가장 마지막 커밋을 베이스로 삼아 병합할 브랜치의 커밋들을 합쳐지게하여 재배치하는 것을 말한다.

Merge와 Rebase의 차이를 그림으로 비교하면 다음과 같다.

다음과 같은 히스토리를 갖고 있다고 가정한다.

<img src="rebase1.png"  width="800" height="400">

다음은 Merge를 이용하여 `experiment`의 커밋 `C4`를 `master` 브랜치에 Merge 하여 결과적으로 `C5` 커밋을 생성한 결과이다. 

<img src="rebase2.png"  width="800" height="300">

반면, 다음은 Rebase를 이용하여 `experiment`의 커밋 `C4`를 `master` 브랜치의 마지막 커밋인 `C3`를 베이스로 삼고 master 브랜치 분기에 `C4'`가 배치된 것을 보여준다. 이는 결과적으로 `experiment`의 베이스가 `C2`에서 `C3`로 바뀌게 된 것이다.

```bash
git checkout experiment
git rebase master
```

<img src="rebase3.png"  width="800" height="300">

## 2. Rebase의 사용

Rebase는 보통 리모트 브랜치에 커밋을 깔끔하게 적용하고 싶을 때 사용한다. 특히 Rebase는 베이스가 다른 브랜치들의 Merge를 용이하게 하기 위해서 사용되기도 한다.

예를 들어 다음과 같은 히스토리를 갖는 저장소가 있다고 가정한다. `server`는 `master`의 `C2` 커밋에서 분기되었다. 반면 `client`는 `server`의 `C3` 커밋에서 분기가 되었다.

<img src="rebase4.png"  width="800" height="450">

`server`와 `client`의 작업을 각각 진행하던 중, `client`의 작업이 완료되어 `master`에 Merge하고자 할 때, `--onto` 옵션을 이용하여 `client`의 커밋인 `C8`, `C9` 뿐만 아니라 `server`와 `client`의 분기인 `C3`를 `master`의 `C6`를 베이스로 하도록 Rebase를 할 수 있다.

```bash
git rebase --onto master server client
```

<img src="rebase5.png"  width="800" height="400">

이 과정에서 물론 충돌이 발생할 수 있으나, 충돌을 해결해주면 Rebase를 할 수 있다. 다음으로 `master`를 `client`에 fast-forward 방식으로 marge를 하면 된다.

```bash
git checkout master
git merge client
```

<img src="rebase6.png"  width="800" height="300">

다음으로 `server` 브랜치를 `master` 브랜치에 Rebase할 수 있다. 물론 Merge하여 두 브랜치를 합치는 것도 가능하지만, 여기서는 Rebase를 활용해 `server`의 커밋 내용을 `master` 브랜치 히스토리에 남기고자 한다.

```bash
git rebase master server
```

<img src="rebase7.png"  width="800" height="100">

그 후 `master` 브랜치와 `server` 브랜치를 Merge하고, 작업이 완료된 `client`, `server` 브랜치는 삭제하면, 히스토리는 다음과 같이 남게 된다.

<img src="rebase8.png"  width="800" height="100">

## Rebase의 위험성

Rebase는 원격 저장소에 push 한 커밋을 Rebase 하는 경우 문제가 발생한다.

Rebase는 기존의 커밋을 그대로 사용하는 것이 아니라 내용은 같지만 다른 커밋을 새로 만든다. 새 커밋을 서버에 Push 하고 동료 중 누군가가 그 커밋을 Pull 해서 작업을 한다고 하자. 그런데 그 커밋을 git rebase 로 바꿔서 Push 해버리면 동료가 다시 Push 했을 때 동료는 다시 Merge 해야 한다. 그리고 동료가 다시 Merge 한 내용을 Pull 하면 내 코드는 정말 엉망이 된다.

따라서 Rebase를 활용하는 경우에는, 팀원과 협의된 상황에서, 코드의 히스토리를 정리할 때에만 팀원들과 함께 작업하는 것이 좋다.