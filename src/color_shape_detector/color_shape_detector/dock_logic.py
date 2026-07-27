"""
도크 표식 인식 순수 로직 — ROS/OpenCV 비의존. cv2 컨투어에서 뽑은 '기하 특징'만 받는다.

왜 순수 함수인가:
  OpenCV 이미지·컨투어를 직접 받으면 배(카메라) 없이는 테스트를 못 한다. 그래서 컨투어에서
  이미 뽑은 숫자(꼭짓점 수, 면적, 둘레, 바운딩박스)만 넘겨받아 판정한다.
  → Mac 에서도 특징값을 넣어 3형상 분류를 검증할 수 있다(test_dock_logic.py).

미션 배경 (도크미션 작업계획 D2·D3):
  표식 = 형상(삼각/원/사각) × 색 조합이고 **당일 아침 공지**된다.
  작년 코드는 목표가 하드코딩(red/Square)이고, 단일 프레임 즉시 발행이라 한 프레임의 오탐이
  그대로 조향에 튀었다. 또 꼭짓점 수 단독 분류라 원/사각 경계가 취약했다.
  → 여기서 (1) 형상 분류를 원형도 보조지표로 강건화하고, (2) N프레임 시간 안정화를 넣는다.
"""

import math

# 형상 이름 (미션 규칙의 3종)
TRIANGLE = "Triangle"
SQUARE = "Square"
CIRCLE = "Circle"
VALID_SHAPES = (TRIANGLE, SQUARE, CIRCLE)

# 색 이름 (5색 슬롯 — 당일 공지 대응)
VALID_COLORS = ("red", "orange", "yellow", "green", "blue", "white")


def circularity(area, perimeter):
    """
    원형도 = 4πA / P².  완전한 원=1.0, 정사각형≈0.785, 정삼각형≈0.605.
    꼭짓점 수만으로 원/사각을 가르면 경계가 취약하다 — 이 값을 보조로 쓴다.
    둘레가 0 이하이면 None.
    """
    if perimeter is None or perimeter <= 0.0:
        return None
    return 4.0 * math.pi * area / (perimeter * perimeter)


def classify_shape(v_count, area, perimeter, w, h, *,
                   min_area=80.0,
                   square_extent_min=0.4,
                   square_aspect_lo=0.6, square_aspect_hi=1.4,
                   circle_circularity_min=0.82,
                   square_circularity_max=0.60):
    """
    컨투어 기하 특징 → 형상 이름 또는 None.

    인자:
      v_count   : approxPolyDP 꼭짓점 수
      area      : 면적(px²)
      perimeter : 둘레(px)
      w, h      : 바운딩박스 폭·높이

    D3 강건화: 꼭짓점 수를 1차로 쓰되, 원/사각 경계는 원형도로 재확인한다.
      · 원형도 >= circle_circularity_min  → 강하게 원
      · 원형도 <  square_circularity_max  → 원이 될 수 없음(각진 도형)
    임계는 전부 파라미터(하드코딩 금지). 실제 값은 D4 하네스 수치로 튜닝한다.
    """
    if area is None or area < min_area:
        return None
    if not v_count or v_count < 3:
        return None
    if w is None or h is None or w <= 0 or h <= 0:
        return None

    circ = circularity(area, perimeter)
    aspect = w / float(h)

    # ---- 삼각형: 꼭짓점 3, 그리고 원형도가 원 수준이 아니어야 ----
    if v_count == 3:
        if circ is not None and circ >= circle_circularity_min:
            return None       # 꼭짓점은 3인데 원형도가 원 → 노이즈. 버린다
        return TRIANGLE

    # ---- 사각형: 꼭짓점 4~6 + 채움비(extent) + 종횡비 + 원형도가 원이 아님 ----
    if 4 <= v_count <= 6:
        extent = area / float(w * h)
        if extent <= square_extent_min:
            return None
        if not (square_aspect_lo <= aspect <= square_aspect_hi):
            return None
        if circ is not None and circ >= circle_circularity_min:
            # 꼭짓점은 사각인데 원형도가 원에 가깝다 → 원일 가능성. 사각으로 확정하지 않는다.
            return CIRCLE
        return SQUARE

    # ---- 원: 꼭짓점 7+ 또는 원형도가 충분히 높음 ----
    if v_count >= 7:
        if circ is not None and circ < square_circularity_max:
            return None       # 꼭짓점 많은데 원형도가 낮다 → 울퉁불퉁한 노이즈. 버린다
        return CIRCLE

    return None


class DetectionConfirmer:
    """
    N프레임 시간 안정화 (D2). 같은 (색,형상) 판정이 연속 confirm_frames 이상일 때만 확정.

    작년: 단일 프레임 판정을 즉시 발행 → 한 프레임 오탐이 그대로 조향에 튐.
    지금: 연속 N프레임 일치해야 확정. 중간에 다른 판정/미검출이 끼면 카운트 리셋.

    사용:
        conf = DetectionConfirmer(confirm_frames=3)
        key = conf.update(("red", "Square"))   # 검출 없으면 None 전달
        if key is not None: 확정 → 발행

    반환:
      확정된 (색,형상) 튜플, 아직 확정 전이면 None.
      한 번 확정되면 같은 판정이 계속되는 한 매번 그 값을 돌려준다(연속 발행).
    """

    def __init__(self, confirm_frames=3):
        self.confirm_frames = max(1, int(confirm_frames))
        self._cur = None      # 지금 세고 있는 후보 (색,형상)
        self._count = 0
        self._confirmed = None

    def update(self, key):
        """key = (색,형상) 또는 None(이번 프레임 미검출)."""
        if key is None:
            # 미검출: 카운트만 깬다. _confirmed 는 유지한다 — 짧은 미검출은 노드의
            # grace_period 가 last_valid 로 덮는다. 다음 검출 재개 시 N프레임을 다시 센다.
            self._cur = None
            self._count = 0
            return None

        if key == self._cur:
            self._count += 1
        else:
            self._cur = key
            self._count = 1

        if self._count >= self.confirm_frames:
            self._confirmed = key
            return key

        # 아직 확정 전 → 이번 프레임엔 아무것도 확정하지 않는다(None).
        #   이전 확정(_confirmed)은 유지된다: 안정된 판정 사이에 오탐 1프레임이 끼어도
        #   그 오탐이 곧장 발행되지 않는다(D2 의 핵심). 오탐이 N프레임 연속돼야 전환된다.
        return None

    @property
    def confirmed(self):
        return self._confirmed
