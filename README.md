# SPRINT

## 이 프로젝트를 시작하는 법

`catkin_ws`의 `src` 안으로 이동하여 아래 명령어로 `git clone`을 수행

```zsh
git clone https://github.com/heely105/SPRINT.git
```

## Git 명령어 치트시트

![Git 작업 흐름](https://www.git-tower.com/learn/content/01-git/01-ebook/en/01-command-line/04-remote-repositories/01-introduction/basic-remote-workflow.png)
[그림 출처](https://www.git-tower.com/learn/git/ebook/en/command-line/remote-repositories/introduction)

### 초기 환경설정 단계

git 처음 쓸 때

- `git config --global user.email "your@email.com"`
- `git config --global user.name "Your Name"`

### clone과 상태 확인하기

- `git clone https://github.com/{소유자}/{저장소}.git`
- `git status`

### pull 단계

- `git pull`

### push 단계

1. `git add .`
2. `git commit -am "{커밋 메시지}"`
    - `git commit`할 때 메시지는 **명령문**로 작성할 것 (예를 들어, `Update function K`)
3. `git push`

### 자동 로그인 설정

방법은 어떤 걸로 선택하든 장단점이 있다.

- 방법 1. 캐시(Cache)를 이용한 방법: [링크](https://franzpark.github.io/git-permanent-authentication/)
- 방법 2. 암호화에 쓸 공개키를 등록하여 자동 로그인: ~~좋은 설명 링크 못 찾음..~~

### branch

![Git 브랜치](https://i.stack.imgur.com/F00b8.png)

1. `git checkout {이동할 브랜치 이름}`
    - 참고로, `checkout`명령은 브랜치를 이동하는 것 뿐만 아니라 (동그라미로 표시된) **과거의 특정지점으로 되돌리는 것도 가능**하다.
2. `git branch {새 브랜치 이름}`
3. `git checkout -b {새 브랜치 이름}`
4. `git merge {가져올 브랜치 이름}`
5. `git branch -d {지우려는 브랜치 이름}`
6. `git push origin :{워크스페이스에서 지운 브랜치 이름}`
7. `git fetch -p`

[리모트 브랜치에 대한 자세한 이해](https://git-scm.com/book/ko/v2/Git-%EB%B8%8C%EB%9E%9C%EC%B9%98-%EB%A6%AC%EB%AA%A8%ED%8A%B8-%EB%B8%8C%EB%9E%9C%EC%B9%98)

### 따로 설명하진 않았지만 기타 명령들

- revert: 이미 커밋한 내용 되돌리기
- reset: 실수로 `merge`한 내용 되돌리기
- 등등...
