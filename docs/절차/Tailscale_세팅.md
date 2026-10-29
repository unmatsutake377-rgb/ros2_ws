# Tailscale 세팅 — 물가 MacBook ↔ 1번배·2번배 노트북 (2026-10-29)

> 왜: 대회 때 배와 물가는 116 m 까지 떨어져 같은 WiFi 를 못 쓴다. 배는 **배에 실은 폰 핫스팟(LTE)**, 물가는 다른 인터넷 —
> 서로 다른 망이라 IP 로 직접 못 닿는다. Tailscale 이 둘을 같은 가상망(100.x.y.z)으로 묶는다. (계획: `소프트_작업체크리스트.md` "0트랙")
> 이 위로 되는 것: **영상(cam_stream → 100.x IP) · SSH**. 🚨 **ROS2 토픽은 안 넘어간다**(DDS 멀티캐스트) — 모니터링은 배에서 돌리고 SSH 로 본다.

## 0. 계정 — 하나로 통일
- **세 기기 모두 같은 계정**으로 로그인해야 같은 망(tailnet)이 된다. 다른 계정이면 서로 안 보인다.
- 무료(Personal) 플랜으로 충분(기기 100대).
- 로그인 수단(Google/GitHub/Microsoft 등)은 팀장 계정 하나로 정하고, 이 문서에 **어느 계정인지만** 적는다(비밀번호·인증키는 저장소에 쓰지 말 것).
  - 사용 계정: ________________

## 1. 기기별 설치

### 1-A. 물가 MacBook
1. 설치 — 둘 중 하나
   - Mac App Store 에서 "Tailscale"
   - 또는 `brew install --cask tailscale`
2. 앱 실행 → 메뉴 막대 Tailscale 아이콘 → **Log in** → 브라우저에서 위 계정으로 로그인.
   (시스템 확장/VPN 구성 허용 팝업이 뜨면 **허용**)
3. 메뉴 막대 아이콘을 누르면 `This Device: <이름> (100.x.y.z)` — 이 **100.x 가 영상 받을 주소**.
4. (선택) 터미널 명령: 메뉴 → Settings → **Install CLI** 후 `tailscale ip -4`, `tailscale status`.

### 1-B. 배 노트북 (1번배·2번배 각각, Ubuntu 22.04)
```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up --hostname=ssf-boat-a        # 2번배는 ssf-boat-b
```
- `sudo tailscale up` 이 **로그인 주소(URL)** 를 출력한다 → 그 노트북 브라우저로 열어 같은 계정 로그인.
- 확인: `tailscale ip -4` (100.x) · `tailscale status` (다른 기기들이 보여야 함).
- 부팅 시 자동 시작은 기본으로 켜져 있다(`systemctl is-enabled tailscaled` → enabled).
- ufw 가 켜져 있으면: `sudo ufw allow in on tailscale0` (Tailscale 망에서 오는 SSH 등 허용).

## 2. 관리 화면에서 한 번 (login.tailscale.com/admin/machines)
- **배 노트북 두 대는 "Disable key expiry"** — 기본 180 일마다 재로그인을 요구한다. 대회 당일 만료되면 SSH·영상이 끊긴다.
- 기기 이름 확인: `ssf-boat-a`, `ssf-boat-b`, MacBook.
- 접근 규칙(ACL)은 기본값(같은 계정 기기끼리 전부 허용) 그대로.

## 3. 주소 기록 (설치 후 채우기 — Tailscale IP 는 기기마다 고정이라 한 번만 적으면 된다)

| 기기 | Tailscale 이름 | 100.x IP |
|---|---|---|
| 물가 MacBook Air | macbookair | 100.93.158.11 |
| 1번배 노트북 (IdeaPad Slim 3 15IAH8) | ssf-boat-a | 100.84.57.20 |
| 2번배 노트북 (IdeaPad Slim 3 **15ABR8** — 1번배와 다른 기종) | 지금 `ssfb-ideapad-slim-3-15abr8` → `sudo tailscale set --hostname=ssf-boat-b` 로 바꿀 것 | 100.119.219.33 |

> 10-29 첫 연결: 세 기기 같은 계정 확인. 1번배 → MacBook `tailscale ping` = `via 192.168.0.1:41641 in 9ms` (**직접 연결**, 같은 공유기).
> 🚨 영상 보낼 때 `cam_stream.sh` 뒤에는 **MacBook 주소(100.93.158.11)**. 배 자기 주소를 넣으면 자기에게 보내 화면이 안 뜬다(첫 시도 때 실제로 그랬다).

## 4. 확인 순서
1. 세 기기 모두 `tailscale status` 에 서로 보인다.
2. 배 노트북에서: `tailscale ping <MacBook 100.x>` — `pong ... via <IP:포트>` 면 **직접 연결**, `via DERP(tok)` 등이면 **중계 서버 경유**(지연 +수십~수백 ms).
3. SSH: MacBook 에서 `ssh <사용자>@ssf-boat-a` (또는 100.x).
4. 영상: MacBook `./tools/cam_view.sh` · 배 `./tools/cam_stream.sh <MacBook 100.x>` — 설명서 `수동조종_영상_설명서.md`.

## 5. 대회 조건 미리 시험 (물 없이 실내에서)
- 배 노트북 → **폰 A 핫스팟**, MacBook → **다른 망**(집 WiFi 또는 폰 B 핫스팟). 일부러 망을 나눈다.
- §4 의 2·4 를 하고 기록: 직접/중계 · 체감 지연 · 깨짐. 깨지면 수신 `CAM_JITTER_MS=150 ./tools/cam_view.sh`.
- 결과는 `docs/결과분석/영상지연_실측_<날짜>.md` 에 "LTE+Tailscale" 행으로.
- ⚠️ 폰 핫스팟 확인: 자동 꺼짐 해제 · 5 GHz(조종기 2.4 GHz 보호) · 데이터 요금(영상 1000 kbps ≈ 시간당 450 MB).

## 되돌리기
- 끊기: `sudo tailscale down` (Mac 은 메뉴에서 Disconnect). 지우기: `sudo apt remove tailscale` / 앱 삭제. 관리 화면에서 기기 Remove.
