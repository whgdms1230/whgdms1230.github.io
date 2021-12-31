---
sort: 1
---

# Command

git을 사용하면서 주로 사용되는 명령어 정리

## init
현재 디렉토리 git 저장소로 초기화
```bash
git init
```

## clone
원격 저장소의 git 복제
```bash
git clone <remote_repository_url>

# 특정 브랜치 clone
git clone <remote_repository_url> -b <branch_name>
```

## remote
원격 저장소 추가
```bash
git remote add <remote_repository_name> <remote_repository_url>
```

## config
저장소 설정
```bash
# 전역 사용자/이메일 설정
git conifg --global user.name "name"
git config --global user.email "email"

# 로컬 사용자/이메일 설정
git config user.name "name"
git config user.email "email"

# 전역 정보 조회
git config --global --list

# 로컬 정보 조회
git config --list
```

## add
변경된 사항 stage 상태로 바꾸기
```bash
# 전체 변경 사항 스테이징
git add -A

# 특정 파일 스테이징
git add <file_name>
```

## commit
stage에 있는 변경사항들 커밋하기
```bash
git commit

# 커밋 메시지 한번에 포함하여 커밋하기
git commit -m "commit message"

# 마지막 커밋 고치기
git commit -m "commit message" --amend
```

## branch
새로운 branch를 생성하고, branch 관리
```bash
# 브랜치 목록 보기
git branch

# 원격 브랜치 목록 보기
git branch -r

# 로컬 및 원격 브랜치 목록 보기
git branch -a

# 새로운 브랜치 생성하기
git branch <branch_name>

# 브랜치 삭제하기
git branch -d <branch_name>
```

## checkout
다른 브렌치로 체크아웃
```bash
git checkout <branch_name>
```

## merge
다른 브랜치를 현재 브랜치로 머지
```bash
git merge <branch_name>

# 커밋하지 않고 머지
git merge --no-commit <branch_name>

# 브랜치 이력을 살린 상태로 머지
git merge --squash <branch_name>
```

## log
깃 이력 확인
```bash
git log

# 패치와 함께 로그 표시
git log -p

# 로그의 항목 개수 N개로 설정
git log -N
```

## diff
파일의 변경내용 확인
```bash
# 최신 commit과 현재 수정상태 비교
git diff

# 최신 commit과 스테이징된 파일 비교
git diff --staged

# commit 간 비교
git diff <commit1_hash> <commit2_hash>

# 브랜치 간 비교
git diff <branch_name_1> <branch_name_2>
```

## fetch
원격 저장소의 변경 사항 가져오기(merge 안됨)
```bash
git fetch

# 특정 원격 저장소의 변경 사항 가져오기
git fetch <remote_repository_name>
```

## pull
원격 저장소의 변경 사항 가져오기(merge 됨)
```bash
git pull

# 특정 원격 저장소의 변경 사항 가져오기
git pull <remote_repository_name>
```

## push
원격 저장소에 로컬 저장소 내용 푸시하기
```bash
# 특정 원격 저장소의 로컬과 동일한 브랜치 이름에 푸시하기
git push <remote_repository_name> <branch_name>

# 특정 원격 저장소의 특정 브랜치 이름에 푸시하기
git push <remote_repository_name> <local_branch_name>:<remote_branch_name>

# 원격 저장소 브랜치 삭제
git push <remote_repository_name> --delete <remote_branch_name>
```