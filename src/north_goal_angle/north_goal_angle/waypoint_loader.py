"""웨이포인트 로드·검증 — ROS 비의존 순수 로직.

대회장에서 GPS 좌표를 바꿀 사람은 config/waypoints.yaml 만 편집한다(코드는 안 건드림).
이 모듈은 그 파일을 읽어 검증한다. **문법·좌표가 이상하면 조용히 넘어가지 않고 예외를 던진다**
— 틀린 좌표로 배가 달리는 것보다 출발 전에 멈추는 게 낫다(모르면 침묵 말고 알린다).

파싱은 순수 함수라 배·ROS 없이 테스트한다(test_waypoint_loader.py).
"""


class WaypointError(ValueError):
    """웨이포인트 파일이 잘못됐을 때. 메시지에 '몇 번째 줄이 왜 틀렸는지'를 담는다."""


# 한국 대회장 대략 범위 — 이 밖이면 좌표 오타로 보고 거부한다.
# (0.0 을 빈칸으로 두거나 위도·경도를 바꿔 넣는 실수를 잡는다)
LAT_MIN, LAT_MAX = 33.0, 39.0     # 한반도 남단~북단 근사
LON_MIN, LON_MAX = 124.0, 132.0

# 미션 번호(wp_mode) — 확정된 값만 허용. 오타(예: 70)를 잡는다.
VALID_MODES = {0, 1, 2, 3, 5, 7, 8}


def parse_waypoints(raw, *, lat_min=LAT_MIN, lat_max=LAT_MAX,
                    lon_min=LON_MIN, lon_max=LON_MAX, valid_modes=VALID_MODES):
    """
    yaml 로 읽은 파이썬 객체(raw) → 검증된 [[lat, lon, mode, dwell], ...].

    raw 는 yaml.safe_load 결과다. 기대 형식:
        {"waypoints": [ {lat:..., lon:..., mode:..., dwell:...}, ... ]}
      또는 각 항목이 리스트 [lat, lon, mode, dwell] 여도 받는다(둘 다 허용).

    잘못되면 WaypointError 를 던진다 — 어느 항목이 왜 틀렸는지 메시지에 담는다.
    검증:
      · 최상위에 'waypoints' 키가 있고 리스트인가
      · 비어있지 않은가
      · 각 항목이 lat/lon/mode/dwell 을 갖는가
      · lat/lon 이 한국 대회장 범위 안인가 (오타·빈칸·위경도 뒤바뀜 탐지)
      · mode 가 허용된 미션 번호인가
      · dwell 이 0 이상 숫자인가
    """
    if not isinstance(raw, dict) or "waypoints" not in raw:
        raise WaypointError(
            "최상위에 'waypoints:' 키가 없다. 파일 첫 줄이 'waypoints:' 인지 확인하라.")

    items = raw["waypoints"]
    if not isinstance(items, list):
        raise WaypointError("'waypoints:' 아래는 목록(-)이어야 한다.")
    if len(items) == 0:
        raise WaypointError("웨이포인트가 하나도 없다. 최소 1개는 있어야 한다.")

    out = []
    for i, it in enumerate(items):
        where = f"{i}번째 웨이포인트"

        # dict 형식({lat:.., lon:.., ...}) 또는 리스트 형식([lat, lon, mode, dwell]) 둘 다 허용
        if isinstance(it, dict):
            missing = [k for k in ("lat", "lon", "mode", "dwell") if k not in it]
            if missing:
                raise WaypointError(f"{where}: {missing} 항목이 빠졌다.")
            lat, lon, mode, dwell = it["lat"], it["lon"], it["mode"], it["dwell"]
            label = it.get("구역", it.get("label", ""))
        elif isinstance(it, (list, tuple)):
            if len(it) != 4:
                raise WaypointError(
                    f"{where}: [위도, 경도, 미션번호, 머무는시간] 4개여야 하는데 {len(it)}개다.")
            lat, lon, mode, dwell = it
            label = ""
        else:
            raise WaypointError(f"{where}: 형식이 이상하다(dict 또는 리스트여야 한다).")

        lat = _as_float(lat, where, "위도(lat)")
        lon = _as_float(lon, where, "경도(lon)")
        dwell = _as_float(dwell, where, "머무는시간(dwell)")

        if not isinstance(mode, int) or isinstance(mode, bool):
            raise WaypointError(f"{where}: 미션번호(mode)는 정수여야 한다 (지금 {mode!r}).")

        # 범위 검증 — 오타·빈칸·위경도 뒤바뀜을 잡는다
        if not (lat_min <= lat <= lat_max):
            raise WaypointError(
                f"{where} '{label}': 위도 {lat} 가 한국 범위({lat_min}~{lat_max}) 밖이다. "
                f"위도·경도를 바꿔 넣었거나 오타가 아닌지 확인하라.")
        if not (lon_min <= lon <= lon_max):
            raise WaypointError(
                f"{where} '{label}': 경도 {lon} 가 한국 범위({lon_min}~{lon_max}) 밖이다.")
        if mode not in valid_modes:
            raise WaypointError(
                f"{where} '{label}': 미션번호 {mode} 는 허용값 {sorted(valid_modes)} 에 없다.")
        if dwell < 0.0:
            raise WaypointError(f"{where} '{label}': 머무는시간 {dwell} 이 음수다.")

        out.append([lat, lon, mode, dwell])

    return out


def _as_float(v, where, name):
    try:
        return float(v)
    except (TypeError, ValueError):
        raise WaypointError(f"{where}: {name} 값 {v!r} 이 숫자가 아니다.")
