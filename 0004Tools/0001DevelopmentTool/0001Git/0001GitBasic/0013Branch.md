---
sort: 13
---

# Branch

## 0. 참고 문헌

*- [Pro Git](https://git-scm.com/book/ko/v2)*

*- [Pro Git-브랜치란 무엇인가](https://git-scm.com/book/ko/v2/Git-%EB%B8%8C%EB%9E%9C%EC%B9%98-%EB%B8%8C%EB%9E%9C%EC%B9%98%EB%9E%80-%EB%AC%B4%EC%97%87%EC%9D%B8%EA%B0%80)*

*- [Pro Git-브랜치 관리](https://git-scm.com/book/ko/v2/Git-%EB%B8%8C%EB%9E%9C%EC%B9%98-%EB%B8%8C%EB%9E%9C%EC%B9%98-%EA%B4%80%EB%A6%AC)*

*- [Pro Git-리모트 브랜치](https://git-scm.com/book/ko/v2/Git-%EB%B8%8C%EB%9E%9C%EC%B9%98-%EB%A6%AC%EB%AA%A8%ED%8A%B8-%EB%B8%8C%EB%9E%9C%EC%B9%98)*

## 1. 브랜치란?

모든 버전 관리 시스템은 브랜치를 지원한다. 개발을 하다 보면 코드를 여러 개로 복사해야 하는 일이 자주 생긴다. 코드를 통째로 복사하고 나서 원래 코드와는 상관없이 독립적으로 개발을 진행할 수 있는데, 이렇게 독립적으로 개발하는 것이 브랜치다.

Git의 브랜치는 커밋 사이를 가볍게 이동할 수 있는 어떤 포인터 같은 것이다. 

> 기본적으로 Git은 master 브랜치를 만든다. 처음 커밋하면 이 master 브랜치가 생성된 커밋을 가리킨다. 이후 커밋을 만들면 master 브랜치는 자동으로 가장 마지막 커밋을 가리킨다.

## 2. Branch

### 2.1 브랜치 생성하기

`branch` 명령으로 브랜치를 생성할 수 있다.

```bash
git branch <브랜치 이름>
```

### 2.2 브랜치 목록 확인하기

다음의 명령으로 현재 레포지토리에 생성된 브랜치의 목록을 모두 확인할 수 있다. 또한, `-v` 옵션을 실행하면 브랜치마다 마지막 커밋 메시지도 함께 보여준다.

```bash
git branch

git branch -v
```

각 브랜치가 지금 어떤 상태인지 확인하기에 좋은 옵션도 있다. 현재 Checkout 한 브랜치를 기준으로 --merged 와 --no-merged 옵션을 사용하여 Merge 된 브랜치인지 그렇지 않은지 필터링해 볼 수 있다.

```bash
git branch --marged

git branch --no-merged
```

커밋이나 브랜치 이름을 지정해주지 않으면 현재 브랜치를 기준으로 Merge 되거나 Merge 되지 않은 내용을 출력한다.

위 명령을 사용할 때 특정 브랜치를 기준으로 Merge 되거나 혹은 Merge 되지 않은 브랜치 정보를 살펴보려면 명령에 브랜치 이름을 지정해주면 된다. 예를 들어 master 브랜치에 아직 Merge되지 않은 브랜치를 살펴보려면 다음과 같은 명령을 실행한다.

```bash
git branch --no-merged master
```

### 2.3 브랜치 삭제하기

`-d` 옵션을 사용하면 브랜치를 삭제할 수 있다. `-D` 옵션을 사용하면 Merge하지 않은 커밋이 있는 브랜치도 강제로 삭제할 수 있다.

```bash
git branch -d <브랜치 이름>

git branch -D <브랜치 이름>
```

리모트 브랜치를 삭제할 때는 `push`명령에 `--delete`옵션을 사용하여 삭제할 수 있다.

```bash
git push <원격 저장소 이름> --delete <브랜치 이름>
```

## 3. checkout

`checkout` 명령으로 다른 브랜치로 이동할 수 있다.

```bash
git checkout <브랜치 이름>
```

## 4. 브랜치 추적

트래킹 브랜치는 리모트 브랜치와 직접적인 연결고리가 있는 로컬 브랜치이다. 트래킹 브랜치에서 git pull 명령을 내리면 리모트 저장소로부터 데이터를 내려받아 연결된 리모트 브랜치와 자동으로 Merge 한다.

서버로부터 저장소를 Clone을 하면 Git은 자동으로 `master` 브랜치를 `origin/master` 브랜치의 트래킹 브랜치로 만든다. 트래킹 브랜치를 직접 만들 수 있는데 리모트를 `origin` 이 아닌 다른 리모트로 할 수도 있고, 브랜치도 `master` 가 아닌 다른 브랜치로 추적하게 할 수 있다.

```bash
git checkout -b <브랜치 이름> <원격 저장소 이름>/<브랜치 이름>
```

`--track` 옵션을 사용하면 로컬 브랜치를 원격 저장소의 브랜치 이름으로 자동으로 생성할 수 있다.

```bash
git checkout --track <원격 저장소 이름>/<브랜치 이름>

# 위 명령은 다음과 같이 원격 저장소의 브랜치 이름으로 checkout 하는 경우와 같음
git checkout <브랜치 이름>
```

리모트 브랜치와 다른 이름으로 브랜치를 만들려면 로컬 브랜치의 이름을 아래와 같이 다르게 지정한다.

```bash
git checkout -b <로컬 브랜치 이름> <원격 저장소 이름>/<원격 브랜치 이름>
```

이미 로컬에 존재하는 브랜치가 리모트의 특정 브랜치를 추적하게 하려면 git branch 명령에 -u 나 --set-upstream-to 옵션을 붙여서 아래와 같이 설정한다.

```bash
git branch -u <원격 저장소 이름>/<브랜치 이름>
```

추적 브랜치가 현재 어떻게 설정되어 있는지 확인하려면 `git branch` 명령에 `-vv` 옵션을 더한다. 이 명령을 실행하면 로컬 브랜치 목록과 로컬 브랜치가 추적하고 있는 리모트 브랜치도 함께 보여준다. 게다가, 로컬 브랜치가 앞서가는지 뒤쳐지는지에 대한 내용도 보여준다.

```bash
git branch -vv
```

> 여기서 중요한 점은 명령을 실행했을 때 나타나는 결과는 모두 마지막으로 서버에서 데이터를 가져온(fetch) 시점을 바탕으로 계산한다는 점이다. 단순히 이 명령만으로는 서버의 최신 데이터를 반영하지는 않으며 로컬에 저장된 서버의 캐시 데이터를 사용한다.