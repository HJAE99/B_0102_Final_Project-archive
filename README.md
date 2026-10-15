# temp/share-pcs — 작업 PC 간 자료 공유용 임시 브랜치

코드와 이력이 없는 빈(orphan) 브랜치다. 각 PC 에만 있는 로그 · 캡처 · 영상 · 코드 조각을 주고받는 데만 쓴다.
main 이나 통합 브랜치에 합치지 않는다. 제출이 끝나면 지운다.

- 폴더: `<PC 이름>/<날짜>/...` (예: `victus/1015/run7_control.log`, `msi/1015/...`, `laptop/1015/...`)
- **공개 저장소다.** 토큰 · 비밀번호 · DB 접속 문자열 · 사람 얼굴이 보이는 사진 · 영상은 올리지 않는다 (가리고 올림).
- 파일 하나 100 MB 넘으면 GitHub 가 거절한다. 큰 영상은 나눠 올리거나 다른 경로로.
- 받는 쪽은 `git fetch origin temp/share-pcs` 뒤 `git show origin/temp/share-pcs:<경로> > 파일` 또는 별도 worktree 로.
