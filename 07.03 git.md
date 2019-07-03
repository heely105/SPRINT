# 작성요령

- `#` 제목

- `-` 소제목

- `1.` 

- `ctrl /` 블럭 처리 후 주석처리
  
# git
- 서버/ ORIGIN / WORKSPACE

# git 요령

- 상태 확인  ```git status```

- 업로드
  1. ```git add .```
  2. ```git commit -am "메세지"```
  3. ```git push```

       ( 유의! ```git pull``` 먼저 실행 )

# branch 만들기
- 새로 만든 파일에 origin 달아주기
  ```git push --set-upstream origin 파일이름```

- 새로 branch 넘어가기 전에는 commit(회색줄 해결하기) 해줘야함

- ```git merge origin/branch이름```

- ```git fetch -p``` 안쓰는 branch 정리

- ```git push origin :삭제할branch``` 서버에서의 branch 삭제


#추가 유의사항
- 자신이 어떤 branch에 있는 지 확인