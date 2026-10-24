# OAK-1 PoE 첫 연결 — 작업 지시 프롬프트 (2026-10-24)

> 대상: 배 노트북(우분투) 세션. `git pull` 뒤 이 파일을 통째로 붙여넣어 시작한다.
> 본문 절차는 `docs/절차/oak_arrival_runbook.md` §1~7 이 원본이다. 이 파일은 **오늘 상태에 맞춘 순서와 전제**만 적는다. 절차가 충돌하면 런북이 맞다.

## 0. 시작 전 — 이게 없으면 못 한다

| 전제 | 상태 (10-24) | 없으면 |
|---|---|---|
| OAK-1 PoE **FF** ×2 | ✅ 10-20 도착 (리얼리, FIXED-FOCUS 69°) | — |
| **PoE 인젝터** (802.3af active, 12-48V 입력) | ⚠️ **옥션 발주분 미도착** (구매 목록 §0-1) | 🚨 **OAK 는 PoE 로만 전원을 받는다.** 밀봉품엔 USB-C 가 없다. 인젝터 없으면 **켤 수 없다.** 사무실에 PoE 스위치/인젝터(802.3af, 48V)가 있으면 벤치용으로 그걸 써도 된다 |
| NETmate NM-UCL02 어댑터 (AX88179, C 플러그) | ✅ 도착 | 노트북 **USB-C 에 직결** (10-24 확인) |
| OAK 동봉 Cat5e 2m | ✅ (OAK 박스 안) | — |
| 2A 퓨즈 + BF303 홀더 | ✅ 2A 10개·홀더 2개 | 벤치에선 퓨즈 없이 인젝터에 벤치 파워(12~24V) 직결 가능 |

**벤치 전원**: 배터리 대신 **벤치 파워서플라이 12~24V** 를 인젝터 DC IN 에. 전류 제한 1A. OAK 는 4~5W.

## 1. 순서 (런북 § 번호)

1. **§1 드라이버**: `sudo apt install ros-humble-depthai-ros`. RealSense 패키지 **지우지 않는다.**
2. **§2 물리**: 인젝터 PoE OUT → Cat5e → OAK. 인젝터 LAN(데이터) → NM-UCL02 → **노트북 USB-C**. `ip link` 로 `enx…` 인터페이스 UP 확인.
3. **§3 고정 IP**: 카메라는 손대지 않는다(공장 폴백 `169.254.1.222`). 노트북 어댑터만:
   ```bash
   nmcli con add type ethernet ifname <enx…> con-name oak-poe ipv4.method manual ipv4.addresses 169.254.1.10/16
   nmcli con up oak-poe && ping -c 3 169.254.1.222
   ```
   ping 안 되면: VPN 끔, `sudo ufw allow 11490/tcp && sudo ufw allow 11491/udp`, 인젝터 PoE LED 확인.
4. **§4 토픽**: 런북의 yaml(`i_ip: 169.254.1.222`, `RGB`, `nn none`, 1280×720 30fps)로 `depthai_ros_driver camera.launch.py`. `ros2 topic list | grep -iE "rgb|color|image"` 로 **실제 이름을 눈으로**. 추측 금지.
   확인한 이름을 **두 곳**: `color_shape_detector/config/vision.yaml` `image_topic` · `ssf_tools/config/ssf_tools.yaml` `image_topic`.
   QoS: 드라이버 발행이 RELIABLE 이든 BEST_EFFORT 든 우리 구독(BEST_EFFORT)은 호환. `ros2 topic hz` 로 30 나오는지.
5. **§5 hfov_deg 실측 — 가장 중요.** FF 모델 사양 HFOV 69° 지만 1280×720 크롭이면 달라진다. 런북 §5 절차(줄자 + 기준물)로 재서 `vision.yaml` `hfov_deg` 에. 검산 절차까지.
6. **§6 HSV**: 실외 부표 색으로 `basic_image_subscriberhsv` 띄워 재캘리브레이션. `min_area_px` 재튜닝.
7. **§7 검증**: 런북 순서대로. 마지막은 메인 launch 로 `ros2 topic info /image_angle` 발행자 1개, `/red_angle`·`/green_angle` hz.

## 2. 커밋 규칙 — 팀 모드 대상이다

`vision.yaml` 의 `image_topic`·`hfov_deg`·HSV 값은 **비전 원자 그룹**(CLAUDE.md §1-1)이고 토픽 이름·파라미터 값이 바뀐다.
→ `ssf-contract` 로 계약서 먼저 → 승인 → 구현 → `ssf-reviewer` + `ssf-verifier`. 운영 규칙 `docs/기준/에이전트팀_운영.md`.
드라이버 설치·IP 설정은 코드가 아니라 계약서 밖. 런북의 "확정값" 칸에 적고 커밋.

## 3. 하지 말 것
- `realsense2_camera` 삭제·`COLCON_IGNORE` 금지 (CLAUDE.md §1-8).
- 카메라 IP 플래시 금지. 노트북 쪽만 고정.
- `CLAUDE.md`, `좌측스러스터_접촉기_20260819.md` 수정 금지. 파일 삭제·rebase·force 금지.
- `python3 -m pytest src/` 금지 → `tools/run_tests.sh`.
- 인젝터 DC IN 극성 반대로 꽂지 말 것 — 벤치 파워 +/− 두 번 본다.

## 4. 끝나면 보고
```
인젝터    : 옥션분 도착 / 사무실 PoE 대체 / 없음(중단)
ping      : 169.254.1.222 OK / FAIL (원인)
컬러 토픽 : <이름>  hz <값>  QoS <값>
hfov_deg  : 실측 ____° (사양 69°, 해상도 ____)
HSV       : 빨강 ____ / 초록 ____  min_area_px ____
/image_angle 발행자 수: __
커밋 해시 : (계약서 → 구현 → 리뷰 통과)
못 한 것과 이유:
```
