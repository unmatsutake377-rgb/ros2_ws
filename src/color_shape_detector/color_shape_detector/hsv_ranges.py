#!/usr/bin/env python3
"""HSV 색 범위 — **단일 출처**. 순수 로직이라 ROS·OpenCV 없이 테스트한다.

🚨 왜 만들었나 (2026-08-12)
   같은 색의 HSV 가 **검출기 3개 파일에 따로 하드코딩**돼 있었고, **값이 서로 달랐다**:

     red    gate/turn S≥140 V≥80   vs  dock S≥80 **V≥200**
     green  gate/turn **H 60~85**  vs  dock **H 28~40**, S≤100
     white  turn 두 색상대(5~33, 75~105) vs dock H 전체 S≤40

   → **같은 초록 부표를 게이트는 보고 도킹은 못 본다.** 어두운 빨강도 마찬가지다.
     에러가 안 나고 그냥 못 찾으므로 물 위에서야 알게 되는 종류다.
     게다가 값을 바꾸려면 **파이썬 코드를 고쳐야** 했다(CLAUDE.md 1-4 하드코딩 금지 위반).

   → 표를 여기 하나로 모으고, 운용값은 `config/vision.yaml` 에서 준다.
     세 검출기가 **같은 패키지 안**이라 공용 모듈을 만들어도 package.xml/setup.py 를
     안 건드린다 — CLAUDE.md 3-9 말미가 경계하는 '패키지 8개 건드리기' 와는 다른 얘기다.

파라미터 표현
   `hsv.<색>` = **정수 배열**, 6개가 한 범위(lo H,S,V + hi H,S,V). 범위 여러 개면 이어붙인다.
       hsv.red: [0,140,80, 5,255,255,  165,140,80, 180,255,255]
   문자열 파싱이나 중첩 구조를 안 쓴다 — ROS2 정수배열이라 검증이 쉽고 yaml 에서 읽기도 낫다.

🚨 **여기 값은 전부 잠정이다.** 실외 실측 전까지 '맞다' 고 믿지 말 것.
   실내 조명에서 잡은 HSV 는 햇빛 아래서 S·V 가 통째로 달라진다.
"""

_VALS_PER_RANGE = 6
H_MAX = 179          # OpenCV 색상은 0~179 (0~360°를 반으로 접음)
SV_MAX = 255


class HsvRangeError(ValueError):
    """HSV 범위가 형식·값 규칙을 어겼다. 조용히 넘기지 않는다."""


# ── 기본 표 (yaml 이 안 닿을 때만 쓰인다) ─────────────────────────────────
#
# 🚨 충돌을 아래 근거로 정리했다. **버린 값도 남긴다** — 나중에 "왜 바꿨지" 를 추적하려고.
#
#  red   : gate/turn 값 채택.  dock 의 V≥200 은 **어두운 빨강을 놓친다**(그늘·흐린 날).
#          버린 값: [0,80,200]-[5,255,255] + [165,80,200]-[180,255,255]
#  green : gate/turn 값 채택.  dock 의 H 28~40 은 초록이 아니라 **연두/노랑**이고,
#          S≤100 이라 **선명한 초록을 오히려 거부**한다. 코드에도 '자리표시자' 라고 적혀 있었다.
#          버린 값: [28,30,235]-[40,100,255]
#  white : dock 값 채택.  '흰색 = 색상 무관 + 저채도 + 고명도' 라는 정의 그대로다.
#          turn 의 두 색상대(5~33, 75~105)는 특정 조명의 색조에 맞춘 흔적으로 보인다.
#          버린 값: [5,2,230]-[33,30,255] + [75,7,60]-[105,45,140]
#  orange/yellow/blue : dock 에만 있던 값 그대로(도크 표식용, 미검증).
# 모양은 검출기가 쓰는 것과 같다: {색: [(lo, hi), …]}  — lo/hi 는 (H, S, V) 3원소.
DEFAULT_RANGES = {
    "red":    [((0, 140, 80), (5, 255, 255)),
               ((165, 140, 80), (180, 255, 255))],
    "green":  [((60, 120, 120), (85, 255, 255))],
    "white":  [((0, 0, 220), (180, 40, 255))],
    "orange": [((3, 130, 100), (20, 255, 255))],
    "yellow": [((21, 120, 60), (37, 255, 255))],
    "blue":   [((130, 18, 160), (175, 60, 200))],
}

# 버린 값 — 실외 튜닝 때 "이쪽이 맞았나" 를 비교할 수 있게 남긴다.
SUPERSEDED = {
    "red@dock":   [0, 80, 200, 5, 255, 255, 165, 80, 200, 180, 255, 255],
    "green@dock": [28, 30, 235, 40, 100, 255],
    "white@turn": [5, 2, 230, 33, 30, 255, 75, 7, 60, 105, 45, 140],
}

VALID_COLORS = tuple(DEFAULT_RANGES)


def flatten(ranges):
    """[(lo, hi), …] → 평평한 정수 리스트(범위당 6개). yaml 에 쓸 때 쓴다.

    lo·hi 가 각각 3원소 튜플이므로 **두 단계**로 편다.
    """
    out = []
    for lo, hi in ranges:
        out.extend(int(v) for v in lo)
        out.extend(int(v) for v in hi)
    return out


def default_flat(color):
    return flatten(DEFAULT_RANGES[color])


def parse_flat(flat, color="?"):
    """평평한 정수 리스트 → [(lo, hi), …]. 규칙을 어기면 HsvRangeError.

    🚨 조용히 고쳐주지 않는다. 잘못된 범위를 '적당히' 받아주면
       물 위에서 "왜 안 잡히지" 를 뒤지게 된다 — 부팅 때 시끄럽게 실패하는 편이 낫다.
    """
    if flat is None:
        raise HsvRangeError(f"[{color}] 값이 없다")
    try:
        vals = [int(v) for v in flat]
    except (TypeError, ValueError):
        raise HsvRangeError(f"[{color}] 정수 배열이어야 한다: {flat!r}")

    if not vals:
        raise HsvRangeError(f"[{color}] 비어 있다")
    if len(vals) % _VALS_PER_RANGE != 0:
        raise HsvRangeError(
            f"[{color}] 길이가 6의 배수여야 한다(범위당 lo3+hi3). 지금 {len(vals)}개")

    ranges = []
    for i in range(0, len(vals), _VALS_PER_RANGE):
        lo = tuple(vals[i:i + 3])
        hi = tuple(vals[i + 3:i + 6])
        _check_triple(lo, color, "lo")
        _check_triple(hi, color, "hi")
        # H 는 색상환이라 lo>hi(빨강처럼 0을 넘어가는 경우)를 두 범위로 나눠 쓰는 게 우리 규약이다.
        # 그래서 H 는 대소를 강제하지 않고, S·V 만 강제한다.
        for idx, name in ((1, "S"), (2, "V")):
            if lo[idx] > hi[idx]:
                raise HsvRangeError(
                    f"[{color}] {name} 하한({lo[idx]})이 상한({hi[idx]})보다 크다")
        ranges.append((lo, hi))
    return ranges


def _check_triple(t, color, which):
    if len(t) != 3:
        raise HsvRangeError(f"[{color}] {which} 는 값 3개여야 한다: {t}")
    h, s, v = t
    if not (0 <= h <= 180):
        raise HsvRangeError(f"[{color}] {which} H={h} 는 0~180 밖이다")
    if not (0 <= s <= SV_MAX) or not (0 <= v <= SV_MAX):
        raise HsvRangeError(f"[{color}] {which} S={s} V={v} 는 0~255 밖이다")


def param_name(color):
    return f"hsv.{color}"


def load(declare_get, colors, on_error=None):
    """검출기용 로더. `{색: [(lo,hi), …]}` 를 돌려준다 — 기존 하드코딩 dict 와 **같은 모양**.

    declare_get(name, default) : 노드의 파라미터 선언+읽기 함수를 주입한다(ROS 비의존 유지).
    on_error(msg)              : 값이 잘못됐을 때 부르는 콜백(보통 logger.error).
                                 잘못된 색은 **기본값으로 되돌린다** — 그 색만 못 쓰는 것보다
                                 낫고, on_error 로 시끄럽게 알린다.
    """
    out = {}
    for c in colors:
        default = default_flat(c)
        raw = declare_get(param_name(c), default)
        try:
            out[c] = parse_flat(raw, c)
        except HsvRangeError as e:
            if on_error:
                on_error(f"🚨 HSV 파라미터 오류 → 기본값 사용: {e}")
            out[c] = parse_flat(default, c)
    return out


def format_yaml(color, ranges, indent=4):
    """튜닝 도구가 찍어주는 yaml 한 줄. 그대로 vision.yaml 에 붙여넣으면 된다."""
    pad = " " * indent
    body = ",  ".join(
        ", ".join(str(v) for v in lo) + ", " + ", ".join(str(v) for v in hi)
        for lo, hi in ranges)
    return f"{pad}{param_name(color)}: [{body}]"


def widen_to_include(ranges, pixel_hsv, margin=(6, 40, 40)):
    """클릭한 픽셀이 들어오도록 **가장 가까운 범위**를 넓힌다.

    튜닝 도구에서 "이 부표를 찍었는데 안 잡힌다" 를 한 번에 해결하려고 쓴다.
    margin 만큼 여유를 준다(조명 변화 흡수).
    """
    if not ranges:
        raise HsvRangeError("넓힐 범위가 없다")
    h, s, v = (int(x) for x in pixel_hsv)

    # H 거리 기준으로 가장 가까운 범위를 고른다 (색상환 wrap 고려)
    def h_dist(r):
        lo, hi = r
        if _in_h(h, lo[0], hi[0]):
            return 0
        return min(_h_gap(h, lo[0]), _h_gap(h, hi[0]))

    idx = min(range(len(ranges)), key=lambda i: h_dist(ranges[i]))
    lo, hi = ranges[idx]
    mh, ms, mv = margin
    new_lo = (max(0, min(lo[0], h - mh)), max(0, min(lo[1], s - ms)), max(0, min(lo[2], v - mv)))
    new_hi = (min(180, max(hi[0], h + mh)), min(SV_MAX, max(hi[1], s + ms)),
              min(SV_MAX, max(hi[2], v + mv)))
    out = list(ranges)
    out[idx] = (new_lo, new_hi)
    return out


def _in_h(h, lo, hi):
    return lo <= h <= hi if lo <= hi else (h >= lo or h <= hi)


def _h_gap(a, b):
    d = abs(a - b)
    return min(d, 181 - d)
