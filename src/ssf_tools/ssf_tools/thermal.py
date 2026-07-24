"""
발열·전원 상태 판정 — 순수 로직. 파일시스템 접근은 호출부가 하고, 값만 넘긴다.

원칙 (검증 피드백 §2-③, CLAUDE.md '거짓 정지 금지'):
  🚨 이 값들은 **경고 로그로만** 쓴다. /health_ok 에 넣지 않는다.
     healthcheck 가 온도로 /health_ok=false 를 만들면 출발 전 진단이 늑대소년이 된다
     (며칠 지나면 팀이 "저건 원래 빨간 거야" 를 배운다).
  우리는 제어 지연을 측정한 적이 없다. "전원 관리가 제어 지연의 원인" 은 미검증 가설이다.
  스로틀링·과열은 '알려진 요인' 이라 **관찰**하고 기록할 뿐, 제어에 개입하지 않는다.

임계는 전부 파라미터로 뺄 수 있게 함수 인자로 받는다(하드코딩 금지, CLAUDE.md 1-4).

sysfs 리더도 여기 둔다 — healthcheck 와 blackbox 가 공유한다.
Ubuntu 아니면 대부분 None 을 돌려준다(Mac/VM 안전).
"""

import glob

# 상태 등급 (경고용 — 제어에 안 쓴다)
LVL_OK = "OK"
LVL_WARN = "WARN"
LVL_HOT = "HOT"


def temp_state(temp_c, *, warn_c=80.0, hot_c=95.0):
    """
    패키지 온도 등급. temp_c 가 None/비정상이면 ('UNKNOWN', None).

    i5-12450H 기준 대략: 상시 80℃ 넘으면 스로틀 임박, 95℃ 부근에서 강제 스로틀.
    ⚠️ 실측값이 아니라 일반적 목표다. 벤치에서 조정할 것.
    """
    if not _finite(temp_c):
        return "UNKNOWN", None
    if temp_c >= hot_c:
        return LVL_HOT, temp_c
    if temp_c >= warn_c:
        return LVL_WARN, temp_c
    return LVL_OK, temp_c


def throttle_detected(cur_khz, max_khz, *, ratio=0.6):
    """
    현재 클럭이 최대의 ratio 아래면 스로틀 의심.

    ⚠️ 이것만으로 스로틀을 단정하지 않는다 — 부하가 없어서 낮을 수도 있다(정상 절전).
       그래서 반환은 '의심' 이고, 거버너를 performance 로 고정한 상태(boat_boot)에서만
       의미가 있다. performance 인데도 클럭이 낮으면 그건 스로틀이다.
    """
    if not (_finite(cur_khz) and _finite(max_khz)) or max_khz <= 0:
        return False, None
    frac = cur_khz / max_khz
    return (frac < ratio), frac


def power_state(ac_online):
    """
    True=AC, False=배터리, None=모름.
    배터리 구동은 경기 전 경고 대상 — 절전 거버너로 내려갈 수 있다.
    """
    if ac_online is None:
        return "UNKNOWN"
    return "AC" if ac_online else "BATTERY"


def summarize(temp_c=None, cur_khz=None, max_khz=None, ac_online=None,
              *, warn_c=80.0, hot_c=95.0, throttle_ratio=0.6):
    """
    한 번에 판정해 dict 로 돌려준다. healthcheck 가 이걸 로그로 찍는다.
    'alert' 는 뭐라도 경고할 게 있으면 True — 하지만 제어엔 안 쓴다.
    """
    tlvl, t = temp_state(temp_c, warn_c=warn_c, hot_c=hot_c)
    thr, frac = throttle_detected(cur_khz, max_khz, ratio=throttle_ratio)
    pwr = power_state(ac_online)

    alert = (tlvl in (LVL_WARN, LVL_HOT)) or thr or (pwr == "BATTERY")
    return {
        "temp_lvl": tlvl,
        "temp_c": t,
        "throttle": thr,
        "clock_frac": frac,
        "power": pwr,
        "alert": alert,
    }


def _finite(x):
    try:
        return x is not None and x == x and abs(x) != float("inf")
    except (TypeError, ValueError):
        return False


# ─────────────────────────────────────────── sysfs 리더 (Ubuntu 전용, 없으면 None)
def _read_int(path):
    try:
        with open(path) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return None


def read_pkg_temp_c():
    """패키지 온도[℃]. hwmon 의 Package 라벨 → thermal_zone 폴백. 없으면 None."""
    for lbl in glob.glob("/sys/class/hwmon/hwmon*/temp*_label"):
        try:
            with open(lbl) as f:
                txt = f.read()
            if "Package" in txt or "pkg" in txt.lower():
                v = _read_int(lbl.replace("_label", "_input"))
                return None if v is None else v / 1000.0
        except OSError:
            continue
    v = _read_int("/sys/class/thermal/thermal_zone0/temp")
    return None if v is None else v / 1000.0


def read_cpu_clock_khz():
    """(현재 최대 코어 클럭, 정격 최대)[kHz]. 못 읽으면 (None, None)."""
    curs = [_read_int(p) for p in
            glob.glob("/sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq")]
    maxs = [_read_int(p) for p in
            glob.glob("/sys/devices/system/cpu/cpu*/cpufreq/cpuinfo_max_freq")]
    curs = [c for c in curs if c is not None]
    maxs = [m for m in maxs if m is not None]
    return (max(curs) if curs else None, max(maxs) if maxs else None)


def read_ac_online():
    """True=AC, False=배터리, None=모름."""
    for p in glob.glob("/sys/class/power_supply/A*/online"):
        v = _read_int(p)
        if v is not None:
            return bool(v)
    return None
