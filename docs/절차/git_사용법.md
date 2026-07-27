# 다른 컴퓨터에서 git 저장소 작업하는 법

> 저장소: **https://github.com/unmatsutake377-rgb/ros2_ws.git** (private)
> "다른 컴퓨터" = 보통 **Ubuntu 빌드 머신**(여기서 실제 colcon build·실행). 규칙은 어느 OS든 같다.

---

## 큰 그림 (30초)

```
GitHub (인터넷 창고, 유일한 진실)
   ▲ push(올림)          ▼ pull(받음)
내 컴퓨터의 ~/ros2_ws (작업 폴더)
```
- **commit** = 내 컴퓨터에 저장 (아직 GitHub엔 안 감)
- **push** = 내 커밋을 GitHub로 올림
- **pull** = GitHub의 남의 변경을 내 컴퓨터로 받음

---

## 1단계. 처음 한 번만 — 받기(clone) + 인증

```bash
# (Ubuntu에 git 없으면) sudo apt update && sudo apt install -y git

git clone https://github.com/unmatsutake377-rgb/ros2_ws.git
cd ros2_ws
```

**private 저장소라 인증을 물어본다:**
- Username: `unmatsutake377-rgb`
- Password: **비밀번호가 아니라 토큰(Personal Access Token)**
  - github.com → 우측 프로필 → Settings → Developer settings → Personal access tokens
    → Tokens(classic) → Generate new token → **`repo` 권한 체크** → 발급된 문자열 복사
  - 이 토큰을 password 자리에 붙여넣는다.

**매번 토큰 다시 치기 싫으면 (한 번만 설정):**
```bash
git config --global credential.helper store   # 토큰을 저장(다음부턴 안 물어봄)
```
> ⚠️ 공용 컴퓨터면 `store` 대신 `cache`(임시)를 쓰거나, 아예 SSH 키를 쓴다.

**(Ubuntu에서만) 처음 빌드:**
```bash
colcon build --symlink-install     # 소스만 받으니 빌드는 새로. build/install/log 는 git에 없음
source install/setup.bash
```

---

## 2단계. 매번 — 이 4줄이 전부다

```bash
cd ~/ros2_ws
git pull                              # ① 시작 전: 다른 컴퓨터가 올린 것 받기

# ... 코드/문서 작업 ...

git add -A                            # ② 바뀐 것 전부 담기
git commit -m "무엇을 왜 바꿨는지"      # ③ 내 컴퓨터에 저장
git push                              # ④ GitHub로 올리기
```

- `git pull` **먼저** 안 하면, 남이 올린 최신 위에서 작업 안 한 게 되어 나중에 충돌난다.
- `git push` **끝나고** 안 하면, 다른 컴퓨터가 내 작업을 못 받는다.

---

## 3단계. 자주 쓰는 확인 명령

```bash
git status            # 지금 뭐가 바뀌었나 / 커밋할 게 있나
git log --oneline -10 # 최근 커밋 10개
git diff              # 아직 커밋 안 한 변경 내용
git remote -v         # 연결된 GitHub 주소 확인
```

---

## 4단계. 자주 겪는 상황

**A. `git pull` 했더니 "conflict"(충돌)가 났다**
→ 두 컴퓨터가 같은 파일의 같은 줄을 고쳤다는 뜻. 파일을 열면 `<<<<<<<`, `=======`, `>>>>>>>` 표시가 있다.
   그 사이에서 **맞는 내용만 남기고 표시들을 지운 뒤**:
```bash
git add -A && git commit -m "충돌 해결"
git push
```
   → **예방이 최선:** 한 번에 한 컴퓨터에서만 작업. 옮길 때 push→pull.

**B. 방금 커밋을 잘못했다 (아직 push 전)**
```bash
git reset --soft HEAD~1    # 마지막 커밋만 취소(변경은 남김) → 고쳐서 다시 커밋
```

**C. 남이 뭘 올렸는지 먼저 보고 싶다**
```bash
git fetch && git log --oneline HEAD..origin/main   # 받기 전에 원격의 새 커밋 미리보기
```

**D. push 했더니 "rejected / non-fast-forward"**
→ 원격에 내가 아직 안 받은 커밋이 있다. `git pull` 로 받고(필요시 충돌 해결) 다시 `git push`.

---

## 🚨 절대 규칙 (이것만 지키면 안 꼬임)

1. **한 번에 한 컴퓨터**에서만 작업. 같은 파일 동시편집 = 충돌.
2. **떠날 때 `push`, 도착해서 `pull`.**
3. 커밋 메시지엔 **무엇을 왜** 바꿨는지. (`git log` 가 팀의 작업 이력이 된다)
4. `build/` `install/` `log/` `*.zip` 은 git에 안 올라간다(`.gitignore`). 각 컴퓨터에서 `colcon build` 로 새로 만든다.
5. **문서 편집은 저장소 `docs/` 에서만.** 바탕화면 보관본은 옛것.

---

## 새 채팅/세션에 넘길 때

`~/ros2_ws/docs/인수인계_현재상태.md` 를 읽히면 지금까지 상태·결정·다음 할 일이 그대로 이어진다.
설계·버그·결정의 단일 출처는 `~/ros2_ws/CLAUDE.md`.
