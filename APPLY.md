# Tech Log. 매거진 홈 리디자인 — 적용 가이드

## 적용 방법
이 폴더의 파일들을 레포 루트에 같은 경로로 덮어쓰기하면 됩니다.
그다음 아래 "삭제할 파일"을 지우고 커밋/푸시하세요.

```
git rm _pages/lorem-ipsum.md _pages/pets.md _pages/recipes-archive.md \
       _pages/sample-page.md _pages/page-a.md _pages/page-b.md \
       _pages/edge-case.md _pages/markup.md _pages/archive-layout-with-content.md \
       _pages/splash-page.md _pages/post-archive-feature-rows.html \
       _pages/collection-archive.html _pages/page-archive.html
```

## 변경 내역

### 새 파일
- `_layouts/home-magazine.html` — 매거진 홈 레이아웃.
  최신 글 1개가 히어로(좌 텍스트 / 우 이미지), 나머지가 2×2 카드 그리드.
  카테고리 필은 site.categories에서 자동 생성되어 /categories/ 앵커로 연결.
- `_sass/minimal-mistakes/skins/_forfun.scss` — 커스텀 스킨.
  크림 배경(#FBFAF8), 오렌지 액센트(#E8470F), 다크 네이비 푸터,
  라이트 페이지 위 다크 코드블록(base16), Noto Sans KR 글로벌 폰트.

### 수정 파일
- `_config.yml` — skin "forfun", name "Myunghwa Lee", description 교체,
  masthead_title "Tech Log.", repository 설정.
- `_data/navigation.yml` — 데모 항목 전부 제거. main: Posts / Categories / Notes(/fun/) / About.
- `index.html` — splash 레이아웃 → home-magazine 레이아웃.
- `_includes/head/custom.html` — Google Fonts(Noto Sans KR 400/500/700/900) 로드 추가.
  기존 MathJax/DataTables는 그대로.
- `assets/css/main.scss` — 다크/splash 시절 규칙(Arial 네비, 오버레이 숨김 등) 제거,
  마스트헤드 로고 스타일과 매거진 홈 스타일(.mag__*) 추가.
  기존 본문 크기/여백 유틸(.ss/.sm/.sl)/프린트 스타일은 유지.

## 포스팅 작성 규칙 (기존과 동일)
- `excerpt`: 히어로/카드 발췌문
- `header.teaser`: 카드 썸네일 (없으면 그라디언트 블록으로 대체)
- `header.image`: 히어로 대형 이미지 (없으면 teaser 사용)
- `categories`: 필터 필과 오렌지 태그

## 참고
- 빌드 테스트 완료 (Jekyll 4.3.2). 기존에 있던 portfolio/index.html과
  _pages/portfolio-archive.md의 destination 충돌 경고는 이번 변경과 무관하게
  원래 있던 것이니 나중에 둘 중 하나로 정리 추천.
- 프린트 스타일이 다크 배경(#252a34) 기준으로 남아있음 — 라이트 테마 전환 후
  PDF 내보내기를 쓰신다면 조정 필요.
