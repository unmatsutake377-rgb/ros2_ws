#!/usr/bin/env python3
"""turn 검출기의 모드별 목표 색 — 순수 로직(ROS·OpenCV 없이 테스트한다).

왜 (계약서_전체코스_20261104 C6 / U5)
   탐색(mode 3)에서 유지할 부표는 **대회 당일 공지되는 색 1개**다. 예전엔
   `wp_color_map = {2: ["white"], 3: ["red"], 4: ["red"], 5: ["red"]}` 가 코드에 박혀 있어
   공지 색이 빨강이 아니면 현장에서 파이썬을 고쳐야 했다(CLAUDE.md 1-4 하드코딩 금지).
   → `turn_color_mode2`·`turn_color_mode3` 파라미터(vision.yaml image_subscriber_turn 절).

허용 색 = 이 검출기가 HSV 범위를 읽어 오는 색(TURN_COLORS)뿐이다.
   process_image 는 color_ranges 에 있는 색만 돈다 — 거기 없는 색(orange 등)을 받아 주면
   에러 없이 **아무것도 못 찾는다.** 그래서 모르는 색은 부팅 때 ERROR 1회 + **그 모드는 검출하지
   않는다**(CLAUDE.md 도크 목표 색 규칙과 같다). 기본값으로 대체하지 않는 이유: 공지 색을
   오타로 적었는데 기본 red 로 조용히 돌면 게이트 빨강 등에 HOLD 할 수 있다.
   → 그 모드에선 /image_angle 을 아예 안 낸다 → (search_behavior hold 기본일 때)
   ship_back SEARCH 20000 유지 — orbit 이면 mode 3 은 ship_turn 담당.
   담당 모드에선 INVALID_LOG_PERIOD_S 마다 ERROR 로 원인을 다시 드러낸다(log_due).
   문자열이 아닌 값(숫자 등)은 rclpy 가 declare_parameter 에서 타입 오류로 부팅 때 노드를 세운다.

🚨 ship_turn(mode 3 orbit)은 /image_color 로 회전 방향을 정한다(빨강·초록=시계,
   노랑·흰색=반시계, 2026-11 경기규정). mode 3 색을 바꾸면 orbit 방향도 바뀐다.
   검정은 위치유지(mode 2) 전용 — 방향이 없어 mode 3 에 쓰면 ship_turn 이 SEARCH 타임아웃.
   그래서 mode 3 에 NO_ORBIT_DIRECTION 색이 오면 노드가 부팅 WARN 을 남긴다(orbit_direction_warning).
   hold(ship_back)는 색으로 방향을 정하지 않는다.
"""

# 이 검출기가 HSV 범위를 읽는 색. basic_image_subscriberturn 의 hsv_ranges.load 와 같은 튜플.
#   2026-11-11: yellow(탐색 좌현선회)·black(위치유지 후보) 추가 — 계약서_탐색선회_부표색_20261111 C1.
TURN_COLORS = ("red", "green", "white", "yellow", "black")

# ship_turn(orbit_logic.orbit_direction_cw)이 선회 방향을 모르는 허용 색. 상수 공유 모듈은 만들지 않는다
#   (CLAUDE.md 1-8) — 바꿀 땐 ship_turn/orbit_logic.py 와 함께.
NO_ORBIT_DIRECTION = ("black",)

# 담당 모드인데 목표 색이 무효일 때 ERROR 를 다시 찍는 주기[s] — dock 의
# TARGET_INVALID_LOG_PERIOD_S 와 같은 값(상수 공유 모듈은 만들지 않는다).
INVALID_LOG_PERIOD_S = 5.0

# 코드 기본값 = vision.yaml(image_subscriber_turn) 값 = 옛 하드코딩 값.
DEFAULT_TURN_COLOR_MODE2 = "white"
DEFAULT_TURN_COLOR_MODE3 = "red"


def resolve_turn_color(raw, allowed=TURN_COLORS):
    """파라미터 값 → (색 또는 None, 오류 메시지 또는 None).

    대소문자·앞뒤 공백은 정리한다(" Red " → "red"). 문자열이 아니거나 허용 목록 밖이면
    (None, 오류 메시지) — 기본값으로 대체하지 않는다. ERROR 로 한 번 찍는 것은 호출자 몫이다.
    """
    if isinstance(raw, str):
        c = raw.strip().lower()
        if c in allowed:
            return c, None
    return None, (f"🚨 turn 목표 색 {raw!r} 는 이 검출기가 모르는 색이다 "
                  f"(허용 {', '.join(allowed)}) → 이 모드는 검출하지 않는다")


def build_wp_color_map(color_mode2, color_mode3):
    """모드 → 목표 색 목록. 모양은 옛 하드코딩 wp_color_map 과 같다.

    4·5 는 mode 3 색을 따른다. active_wp_modes [2, 3] 이라 지금은 모드 게이트가 먼저 막아
    쓰이지 않는다(옛 값도 mode 3 과 같은 red 였다). 기본값일 때 옛 맵과 똑같다.
    색이 None(무효)이면 그 모드는 **빈 목록**이다 — 키를 빼면 target_colors_for 가
    '모든 색' 으로 떨어지므로 키는 반드시 남긴다.
    """
    def one(c):
        return [] if c is None else [c]
    return {
        2: one(color_mode2),
        3: one(color_mode3),
        4: one(color_mode3),
        5: one(color_mode3),
    }


def target_colors_for(wp_mode, wp_color_map, all_colors):
    """process_image 가 이번 모드에서 볼 색. 맵에 없는 모드는 모든 색(옛 동작).

    맵에 있는데 빈 목록(무효 색)이면 빈 목록 → 어떤 색도 통과하지 않는다.
    """
    if wp_mode in wp_color_map:
        return list(wp_color_map[wp_mode])
    return list(all_colors)


def detection_disabled(wp_mode, wp_color_map):
    """이 모드의 목표 색이 무효라 검출을 끈 모드인가 — 참이면 발행(fallback 포함)도 하지 않는다."""
    return wp_mode in wp_color_map and not wp_color_map[wp_mode]


def log_due(last, now, period=INVALID_LOG_PERIOD_S):
    """주기 로그를 지금 찍을 차례인가. last=None(아직 안 찍음)이면 바로 참.

    시각은 time.monotonic() 값이어야 한다(CLAUDE.md 1-5). 경계(정확히 period)는 거짓 —
    dock 의 `(now - last) > period` 와 같은 판정.
    """
    return last is None or (now - last) > period


def orbit_direction_warning(color_mode3):
    """mode 3 목표 색이 ship_turn 이 방향을 모르는 색이면 WARN 문구, 아니면 None.

    무효 색(None)은 resolve_turn_color 가 이미 ERROR 로 알리므로 여기선 None.
    """
    if color_mode3 in NO_ORBIT_DIRECTION:
        return (f"⚠️ turn_color_mode3={color_mode3!r} — ship_turn(orbit)이 이 색의 선회 방향을 모른다 "
                f"→ 탐색에서 SEARCH 타임아웃. 탐색 부표는 red/green(시계)·yellow(반시계). "
                f"{color_mode3} 는 위치유지(turn_color_mode2)용")
    return None
