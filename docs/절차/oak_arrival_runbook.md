# OAK-1 W PoE 도착일 런북

> **목표: 도착일에 남는 작업이 "yaml 수정 + HSV 재캘리브레이션" 뿐이 되게 한다.**
> 코드는 V1~V5 에서 이미 중립화했다. 이 문서는 실물이 온 뒤의 절차다.
>
> 작성 2026-07-23 (카메라 **미도착** 상태). ⚠️ 표시는 실물 없이 확정 못 한 것 —
> 추측으로 채우지 않았다. 도착일에 채울 것.

---

## 0. 지금 상태 (도착 전)

| 항목 | 상태 |
|---|---|
| RealSense D455 | **현역.** `src/realsense-ros-ros2-master/` 유지 — 삭제·`COLCON_IGNORE` 금지 |
| 뎁스 의존 | **이미 끊었다** (V1). 비전 노드는 컬러만 쓴다 |
| 이미지 토픽 | **파라미터** `image_topic` (V5) |
| 화각 | **파라미터** `hfov_deg` (V3) |
| 디버그 창 | **파라미터** `debug_view`, 기본 false (V4) |

즉 **코드는 안 고친다.** 아래는 설정과 캘리브레이션 절차다.

---

## 1. 드라이버 설치 (교체 아님 — **병행**)

`realsense2_camera` 를 **지우지 않는다.** OAK 가 안 되면 되돌아갈 곳이 필요하다.

```bash
sudo apt update
sudo apt install ros-humble-depthai-ros
```

⚠️ apt 패키지가 없거나 버전이 안 맞으면 소스 빌드로 간다. 그 경우 **워크스페이스에 내재화**할 것 —
`git clone` 후 `.git` 을 지우고 일반 디렉터리로 커밋한다. 서브모듈로 두면 다른 머신에서
**빈 폴더**로 온다 (실제로 rplidar_ros·micro_ros 4개가 그 상태였다. `docs/` 의 내재화 커밋 참고).

---

## 2. PoE 물리 연결

OAK-1 W **PoE** 모델은 USB 가 아니라 **이더넷**이다.

1. PoE 인젝터(또는 PoE 스위치) → 카메라
2. 인젝터의 데이터 포트 → 노트북 이더넷
3. 노트북에 이더넷 포트가 없으면 **USB-C 이더넷 어댑터**.
   ⚠️ **기가비트** 어댑터를 쓸 것. 100Mbps 어댑터는 대역폭이 모자라 프레임이 끊긴다.

### 연결 확인

```bash
ip link                       # 이더넷 인터페이스가 UP 인가
ping <카메라IP>                # ⚠️ IP 는 도착일에 확인 (아래 3 참고)
```

---

## 3. 고정 IP

DHCP 로 두면 재부팅마다 IP 가 바뀌어 launch 가 조용히 실패한다. **고정 IP 로 박는다.**

⚠️ 카메라 기본 IP·설정 방법은 실물 문서/툴로 확인해야 한다(Luxonis 문서).
확인 후 **여기에 적을 것**:

```
카메라 IP   : __________
노트북 IP   : __________
서브넷      : __________
설정 방법   : __________
```

---

## 4. 토픽 이름 확인 → yaml 수정

드라이버를 띄우고 **실제 컬러 토픽 이름을 눈으로 확인**한다. 추측 금지.

```bash
ros2 launch depthai_ros_driver camera.launch.py
ros2 topic list | grep -i color
ros2 topic hz <컬러토픽>          # 실제로 나오는지
ros2 topic info <컬러토픽> --verbose   # QoS 확인
```

확인한 이름을 **두 곳** 에 넣는다 — 한쪽만 고치면 조용히 어긋난다:

| 파일 | 항목 | 왜 |
|---|---|---|
| `src/color_shape_detector/config/vision.yaml` | `image_topic` | 검출 노드용 (`/**:` 블록 — 3종 전부에 적용) |
| `src/ssf_tools/config/ssf_tools.yaml` | `image_topic` | `healthcheck` 의 카메라 침묵 감지용 |

✅ **3-4 이후로는 한 곳만 고치면 된다.** 예전엔 `subscriber_mode_manager` 가 검출기를
`ros2 run` 으로 띄워서 launch/yaml 이 안 닿았고, 매니저가 중계하는 `vision_*` 를 따로
고쳐야 했다. 매니저를 폐기하고 launch 가 검출기를 직접 띄우므로 **yaml 이 그대로 닿는다.**

### QoS 주의

비전 노드 구독은 **BEST_EFFORT + depth=1** 이다(V4).
구독자 BEST_EFFORT 는 발행자가 RELIABLE 이든 BEST_EFFORT 든 **전부 호환**되므로
드라이버가 어느 쪽이어도 안전하다. (반대 조합 — 구독자 RELIABLE + 발행자 BEST_EFFORT — 이
**0건**이 되는 함정인데, 우리는 그 방향이 아니다.)

---

## 5. 화각(`hfov_deg`) 실측 — **가장 중요**

OAK-1 W 는 광각(120~150° DFOV)이다. RealSense(71.5°)와 **같은 픽셀이 완전히 다른 각도**다.
계산해 보면 71.5° → 120° 로 바뀔 때 같은 픽셀의 각도가 **최대 24° 달라진다.**
이걸 안 고치면 물 위 튜닝이 통째로 무효가 된다.

### 실측 절차

카탈로그의 DFOV 를 그대로 넣지 마라 — 우리가 쓰는 건 **HFOV(수평)** 이고, 해상도·크롭에 따라 다르다.

⚠️ **거리를 먼저 정하라 — 광각일수록 가까이 가야 한다.** 2m 에서 필요한 좌우폭:

| HFOV | 2.0m 에서 필요한 폭 | 실내에서 가능? |
|---|---|---|
| 71.5° (RealSense) | 2.88m | ○ |
| 120° | 6.93m | △ 넓은 방 |
| 150° | **14.93m** | ✗ **불가능** |

150° 면 `D=0.5m` 로 재라 (필요 폭 3.73m). 대신 **거리 측정 오차의 영향이 커진다** —
자로 `D` 를 정확히 재고, 카메라 **렌즈면**(몸체 앞면 아님) 기준으로 잴 것.

1. 벽에서 정한 거리 **D** 만큼 떨어뜨려 카메라를 벽과 수직으로 놓는다
2. 화면 **좌우 끝**에 각각 걸치는 지점을 벽에 표시 (`debug_view:=true` 로 화면을 보며)
3. 두 표시 사이 거리 `W` 를 자로 잰다
4. `hfov_deg = 2 * degrees(atan((W/2) / D))`

```bash
python3 -c "
import math
W = float(input('좌우 표시 간 거리 W[m]: '))
D = float(input('벽까지 거리 D[m]: '))
print(f'hfov_deg = {2*math.degrees(math.atan((W/2)/D)):.2f}')
"
```

측정값을 `vision.yaml` 의 `hfov_deg` 에 넣는다 (`/**:` 블록이라 검출기 3종에 함께 적용된다).

### 검산

`fx = (width/2)/tan(hfov/2)`. `camera_info` 토픽이 주는 `fx` 와 비교한다:

```bash
ros2 topic echo <camera_info토픽> --once
```

두 값이 크게 다르면 측정이 틀렸거나 해상도 설정이 다른 것이다. **맞을 때까지 진행하지 마라.**

### 🚨 광각은 핀홀 모델이 깨진다

우리 `angle_from_pixel()` 은 **핀홀 모델**이다. 광각 렌즈는 가장자리에서 왜곡이 커서
`hfov_deg` 만 바꾸는 걸로 **부족하다.** 둘 중 하나를 해야 한다:

- **rectified(왜곡보정) 이미지 토픽을 구독** — `image_topic` 을 그쪽으로 (제일 간단)
- `camera_info` 의 왜곡계수 `D` 를 `cv2.undistortPoints` 로 적용 (코드 변경 필요)

가장자리 표식의 방위가 몇 도씩 틀어져도 **에러가 안 난다.** 조용히 빗나갈 뿐이다.
격자 종이를 찍어 직선이 휘는지 눈으로 확인할 것.

---

## 6. HSV 재캘리브레이션

센서·렌즈·화이트밸런스가 다르면 **같은 부표가 다른 HSV 로 찍힌다.** 반드시 다시 잡는다.

```bash
ros2 run color_shape_detector basic_image_subscriberhsv \
  --ros-args -p image_topic:=<컬러토픽>
```

마우스를 올리면 그 픽셀의 HSV 가 터미널에 찍힌다.
(이 노드만 `debug_view` 기본 `true` 다 — 창이 존재 이유라서.)

1. **실제 부표**를 **대회장과 비슷한 조명**에서 찍는다. 실내 형광등에서 잡은 값은 물 위에서 안 맞는다
2. 빨강/초록 각각 여러 지점·여러 거리에서 HSV 범위를 읽는다
3. 여유를 두고 범위를 정한다 (물보라·역광·그림자에서 값이 흔들린다)
4. `basic_image_subscriber{gate,dock,turn}.py` 의 `color_ranges` 에 반영

⚠️ 빨강은 **HSV 색상환의 0/180 경계에 걸쳐** 두 구간으로 나뉜다. 기존 코드가 이미 그렇게 돼 있다:
```python
"red": [([0, 140, 80], [5, 255, 255]), ([165, 140, 80], [180, 255, 255])]
```
한 구간만 남기면 절반을 놓친다.

### `min_area_px` 재튜닝

광각은 각도 해상도가 낮아 **같은 거리의 표식이 더 작게** 보인다.
면적 임계(`area < 40`, `area < 80`)를 낮춰야 할 수 있다. 실제 부표를 목표 거리에 놓고 확인.

---

## 7. 검증 순서 (이 순서대로)

각 단계가 통과해야 다음으로 간다. 건너뛰면 어디서 틀렸는지 못 찾는다.

| # | 확인 | 명령 / 기준 |
|---|---|---|
| 1 | 이미지가 나오나 | `ros2 topic hz <컬러토픽>` → 목표 fps 근처 |
| 2 | 노드가 이미지를 받나 | `ros2 run ... basic_image_subscribergate --ros-args -p image_topic:=<토픽> -p debug_view:=true` |
| 3 | 각도 토픽이 나오나 | `ros2 topic hz /red_angle` — **V1 회귀 확인 지점** |
| 4 | 각도가 맞나 | 표식을 **화면 정중앙** 에 두고 `/red_angle` ≈ **0** |
| 5 | 부호가 맞나 | 표식을 **왼쪽** 으로 → 각도 **양수**. 오른쪽 → 음수. **뒤집히면 조향이 반대로 돈다** |
| 6 | 화각이 맞나 | 표식을 화면 **좌측 끝** 으로 → 각도 ≈ `hfov_deg / 2` |
| 7 | healthcheck 가 보나 | `ros2 topic echo /health_ok` → `CAM/image ✅` |
| 8 | 헤드리스에서 사나 | `debug_view:=false` 로 SSH 에서 실행 → 안 죽나 |

**4~6 이 이 런북의 핵심이다.** 여기서 틀리면 그 뒤 물 위 튜닝이 전부 무효다.

---

## 8. 롤백

OAK 가 안 되면 **yaml 만 되돌리면** RealSense 로 돌아온다. 코드 변경 없음.

```yaml
# vision.yaml  (/**: 블록)
image_topic: "/camera/camera/color/image_raw"
hfov_deg: 71.5
# ssf_tools.yaml  (/healthcheck)
image_topic: "/camera/camera/color/image_raw"
```

HSV 는 **되돌릴 값을 미리 저장해 둘 것.** 재캘리브레이션 전에 기존 `color_ranges` 를 복사해 두면
롤백이 30초 만에 끝난다. 안 해두면 대회장에서 다시 잡아야 한다.

---

## 9. 도착일 체크리스트 (인쇄용)

- [ ] `depthai-ros` 설치 (RealSense 드라이버 **유지** 확인)
- [ ] PoE 인젝터 + **기가비트** 이더넷 연결
- [ ] 고정 IP 설정, `ping` 통과
- [ ] 컬러 토픽 이름 확인 (`ros2 topic list`)
- [ ] `vision.yaml` — `image_topic`
- [ ] `ssf_tools.yaml` — `image_topic`
- [ ] 화각 실측 → `hfov_deg`
- [ ] `camera_info` 의 `fx` 로 검산
- [ ] 왜곡 대응 결정 (rectified 토픽 vs `undistortPoints`)
- [ ] 기존 HSV 값 **백업**
- [ ] HSV 재캘리브레이션 (대회장 유사 조명)
- [ ] `min_area_px` 확인
- [ ] `active_wp_modes` 확인 (3-4: gate=[0,1], turn=[2,3], dock=[7])
- [ ] `ros2 topic info /image_angle` — 발행자가 **1개**인지 (2개면 모드 겹침)
- [ ] 검증 1~8 전부 통과
- [ ] `debug_view: false` 로 되돌렸는지 (대회 설정)
