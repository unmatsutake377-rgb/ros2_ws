#!/usr/bin/env python3
"""mission_monitor 의 순수 로직 — ROS 없이 테스트한다.

화면에 뭘 어떻게 찍을지만 담는다. 구독·타이머는 mission_monitor.py 가 한다.

설계 원칙은 이 저장소의 나머지와 같다 — **모르면 입을 다문다.**
값이 안 왔거나 묵었으면 `—` 를 찍는다. 마지막 값을 계속 보여주면
"3분 전에 자율이었다" 를 "지금 자율이다" 로 읽게 된다. 그건 위험하다.
"""

# wp_mode → 사람이 읽는 미션 이름 (CLAUDE.md 3-6 실제 웨이포인트 표)
#   실제로 나오는 값은 {0,1,2,3,5,7,8} 뿐이다.
#   5·8 은 담당 노드가 없는 게 정상이다(순수 회피 구간) — '누락' 이 아니다.
MISSION_NAMES = {
    0: "게이트 시작",
    1: "게이트 통과",
    2: "위치 유지",
    3: "부표 선회",
    5: "회피 구간",
    7: "도킹",
    8: "토너먼트 회피",
}

# /boat_mode — 펌웨어 enum 과 값이 같다 (ssf_bridge.status_parser 와 동일)
BOAT_MODE_NAMES = {0: "⚪ 대기", 1: "🟢 수동 (RC)", 2: "🟡 자율 (AUTO)"}
BOAT_ID_NAMES = {0: "A", 1: "B", 2: "🚨 FAULT"}

# 페일세이프 레벨 (CLAUDE.md 5장)
FAILSAFE_NAMES = {0: "정상", 1: "⚠️ 감속", 2: "🚨 정지"}

# 수평 정확도(σ, m) → RTK 품질 라벨.
#   🚨 이건 **공분산에서 추정한 것**이지 carrSoln 을 읽은 게 아니다.
#      진짜 FIXED 확인은 /ublox_gps_node/navpvt 의 carrSoln==2 다.
#      여기서 그걸 안 읽는 이유: ublox_msgs 의존을 ssf_tools 에 추가하면
#      빌드 위험이 늘고, 대회 전에 패키지 의존을 늘리지 않기로 했다(CLAUDE.md 3-9 말미).
#   근거 실측(2026-08-07 야외): FIXED 에서 ±0.015m, float 구간에서 ±0.10m.
RTK_FIXED_MAX_SIGMA = 0.05
RTK_FLOAT_MAX_SIGMA = 0.50

_DASH = "—"


def mission_name(wp_mode):
    """wp_mode → 이름. 표에 없으면 번호를 드러낸다(숨기면 추적을 못 한다)."""
    if wp_mode is None:
        return _DASH
    return MISSION_NAMES.get(wp_mode, f"🚨 미정의 모드 {wp_mode}")


def boat_mode_name(mode):
    if mode is None:
        return _DASH
    return BOAT_MODE_NAMES.get(mode, f"🚨 알수없음({mode})")


def rtk_label(sigma_m):
    """수평 σ(m) → 품질 라벨. None 이면 침묵."""
    if sigma_m is None:
        return _DASH
    if sigma_m <= RTK_FIXED_MAX_SIGMA:
        return f"✅ FIXED 추정  ±{sigma_m*100:.1f}cm"
    if sigma_m <= RTK_FLOAT_MAX_SIGMA:
        return f"🟡 float 추정  ±{sigma_m*100:.0f}cm"
    return f"🔴 단독측위  ±{sigma_m:.1f}m"


def fmt_dist(m):
    if m is None:
        return _DASH
    return f"{m:.1f} m"


def fmt_secs(s):
    if s is None:
        return _DASH
    return f"{s:.0f} s"


def fmt_bool_alarm(v, true_text, false_text):
    """True 가 '나쁨' 인 불리언 표시."""
    if v is None:
        return _DASH
    return true_text if v else false_text


def render(state, width=54):
    """상태 dict → 화면 문자열.

    state 키는 전부 없거나 None 일 수 있다(아직 안 왔거나 묵었음).
      boat_mode, boat_id, watchdog, estop,
      wp_mode, goal_dist, wp_remain,
      rtk_sigma, failsafe, gates, health_ok, bridge_seen
    """
    g = state.get
    bar = "─" * width

    lines = [bar, "  SSF 미션 모니터".ljust(width), bar]

    # ── 배 상태(펌웨어) ─────────────────────────────────────────
    if not g("bridge_seen"):
        # 브릿지가 없으면 모드를 '모른다'. 빈칸으로 두면 "수동인가보다" 로 읽힌다.
        lines.append("  모드      : — (브릿지 미연결 — 배 LED 로 확인)")
    elif g("boat_mode") is None:
        # 🚨 받다가 끊긴 것은 '아직 안 옴' 과 다른 **사건**이다.
        #    그냥 — 로 두면 조용히 넘어간다. 브릿지가 죽었으면 자율 명령도 안 나간다는 뜻이고,
        #    펌웨어 워치독(500ms)이 곧 중립으로 잡는다 — 배가 곧 멈춘다는 예고다.
        lines.append("  모드      : 🚨 브릿지 끊김 (자율 명령 중단 → 곧 중립)")
    else:
        lines.append(f"  모드      : {boat_mode_name(g('boat_mode'))}"
                     f"   배 {BOAT_ID_NAMES.get(g('boat_id'), _DASH)}")
        lines.append(f"  명령      : "
                     f"{fmt_bool_alarm(g('watchdog'), '🚨 끊김 → 중립', '정상')}")
        if g("estop"):
            lines.append("  비상정지  : 🚨 눌림")

    lines.append("")

    # ── 미션 ────────────────────────────────────────────────────
    wp = g("wp_mode")
    lines.append(f"  현재 미션 : [{wp if wp is not None else _DASH}] {mission_name(wp)}")
    lines.append(f"  목표까지  : {fmt_dist(g('goal_dist'))}"
                 f"    남은시간 {fmt_secs(g('wp_remain'))}")
    lines.append(f"  게이트    : {g('gates') if g('gates') is not None else _DASH} 통과")

    lines.append("")

    # ── 안전 ────────────────────────────────────────────────────
    fs = g("failsafe")
    lines.append(f"  페일세이프: {FAILSAFE_NAMES.get(fs, _DASH) if fs is not None else _DASH}")
    lines.append(f"  위치정확도: {rtk_label(g('rtk_sigma'))}")
    ok = g("health_ok")
    lines.append(f"  health_ok : {_DASH if ok is None else ('✅ true' if ok else '🚨 false')}")

    lines.append(bar)
    return "\n".join(lines)
