#!/usr/bin/env python3
"""check_params — config yaml 의 값이 **실제로 노드에 닿았는지** 대조한다.

    (전체 launch 를 띄운 상태에서)
    python3 tools/check_params.py

왜 만들었나 (2026-08-12)
  `rgb_camera.controls.saturation: 120` 이 **존재하지 않는 이름**이라 조용히 무시되고 있었다.
  넉 달 동안 아무도 몰랐다 — 에러가 안 나기 때문이다. 같은 유형을 하루에 3건 잡았다.
  ROS2 는 선언 안 된 파라미터를 override 로 받아만 두고 **쓰지 않는다.** 경고도 없다.

무엇을 잡나
  ① **미선언** — yaml 의 노드 전용 절에 있는데 노드가 선언한 적 없는 이름. 오타이거나 죽은 설정.
  ② **불일치** — 선언은 됐는데 값이 yaml 과 다르다 = **yaml 이 안 실렸다**
                (launch 가 `parameters=` 를 안 넘긴 경우가 대표적). 제일 위험하다.

⚠️ **사각지대** — yaml 이 안 실렸는데 그 값이 **코드 기본값과 같으면** 이 도구는 못 잡는다.
   "불일치 0" 은 "yaml 이 다 실렸다" 가 아니라 "실린 것과 다른 값이 없다" 는 뜻이다.

🚨 이 도구를 만들며 **내가 낸 오탐 3종** — 같은 실수를 반복하지 않으려고 적어둔다:
  · `/**`(공통 절)을 이름만 보고 아무 노드에나 적용 → vision.yaml 을 받지도 않는
    RealSense 노드에 '미선언 11건'. → **그 키를 하나라도 선언한 노드만** 대상으로 본다.
  · 노드 전용 절이 공통 절을 **덮어쓴다**는 걸 빼먹음 → dock 의 의도된 재정의(80/3)를
    '불일치' 로 보고. → 공통 위에 전용을 덮어서 비교한다.
  · **실패한 dump 를 빈 값으로 캐시** → ship_gate 의 정상 파라미터 11개를 '미선언' 으로 보고.
    → 빈 결과는 캐시하지 않고 재시도한다. **도구가 조용히 틀리면 없느니만 못하다.**
"""

import argparse
import subprocess
import sys

import yaml

SKIP_SECTIONS = {"waypoints", "ntrip"}       # 파라미터가 아니라 데이터
SKIP_PARAMS = {"use_sim_time"}

DEFAULT_FILES = [
    "src/color_shape_detector/config/vision.yaml",
    "src/ssf_heading/config/ssf_heading.yaml",
    "src/north_goal_angle/config/north_goal_angle.yaml",
    "src/ship_direction/config/ship_direction.yaml",
    "src/ship_goal_angle/config/ship_goal_angle.yaml",
    "src/motor_control/config/motor_control.yaml",
    "src/ship_gate/config/ship_gate.yaml",
    "src/ship_dock/config/ship_dock.yaml",
    "src/ship_turn/config/ship_turn.yaml",
    "src/ship_back/config/ship_back.yaml",
    "src/ssf_tools/config/ssf_tools.yaml",
    "src/ublox/ublox_gps/config/zed_f9p_rover.yaml",
]


def sh(cmd, timeout=40):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip()
    except Exception:                        # noqa: BLE001
        return ""


def flatten(d, prefix=""):
    """중첩 dict → 점 표기 (`gnss: {glonass: true}` → `gnss.glonass`)."""
    out = {}
    for k, v in d.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(flatten(v, key + "."))
        else:
            out[key] = v
    return out


class Live:
    """노드의 실제 파라미터. 🚨 **빈 결과는 캐시하지 않는다** — 실패를 사실로 굳히지 않는다."""

    def __init__(self):
        self._cache = {}

    def params(self, node):
        if self._cache.get(node):
            return self._cache[node]
        for _ in range(2):                   # 한 번 실패하면 재시도
            out = sh(["ros2", "param", "dump", node])
            if out:
                try:
                    doc = yaml.safe_load(out) or {}
                except Exception:            # noqa: BLE001
                    doc = {}
                for _k, v in doc.items():
                    if isinstance(v, dict) and isinstance(v.get("ros__parameters"), dict):
                        p = flatten(v["ros__parameters"])
                        self._cache[node] = p
                        return p
        return {}                            # 캐시하지 않음


def same(expected, actual):
    """yaml 값과 실제 값을 느슨하게 비교(정수/실수 혼용 허용)."""
    if actual is None:
        return False
    if isinstance(expected, bool) or isinstance(actual, bool):
        return bool(expected) == bool(actual)
    if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        return abs(float(expected) - float(actual)) < 1e-6
    if isinstance(expected, list) and isinstance(actual, list):
        return len(expected) == len(actual) and all(same(e, g)
                                                    for e, g in zip(expected, actual))
    return str(expected) == str(actual)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yamls', nargs='*')
    args = ap.parse_args()
    files = args.yamls or DEFAULT_FILES

    nodes = [n for n in sh(["ros2", "node", "list"]).splitlines() if n.startswith("/")]
    if not nodes:
        print("🚨 떠 있는 노드가 없다. 전체 launch 를 먼저 띄울 것.")
        return 1
    print(f"떠 있는 노드 {len(nodes)}개\n")

    live = Live()
    n_undecl = n_mismatch = n_absent = n_nodump = n_note = 0

    for f in files:
        try:
            doc = yaml.safe_load(open(f, encoding='utf-8')) or {}
        except Exception as e:               # noqa: BLE001
            print(f"⚠️ {f}: 읽기 실패 {e}")
            continue
        if not isinstance(doc, dict):
            continue

        common = {}
        cs = doc.get("/**")
        if isinstance(cs, dict) and isinstance(cs.get("ros__parameters"), dict):
            common = flatten(cs["ros__parameters"])

        for sect, body in doc.items():
            if sect in SKIP_SECTIONS or not isinstance(body, dict):
                continue
            params = body.get("ros__parameters")
            if not isinstance(params, dict):
                continue
            own = {k: v for k, v in flatten(params).items() if k not in SKIP_PARAMS}

            if sect == "/**":
                # 공통 절: 그 키를 **하나라도 선언한 노드**만 이 yaml 을 받은 것으로 본다
                targets = [n for n in nodes if any(k in live.params(n) for k in own)]
                own_keys = set()             # 공통 키의 미선언은 정상(노드마다 쓰는 색이 다름)
                merged = own
                # 🚨 노드 전용 절이 있는 노드는 **여기서 검사하지 않는다.**
                #    그 절이 공통값을 덮으므로, 공통값과 비교하면 의도된 재정의를
                #    '불일치' 로 잘못 보고한다(dock 의 80/3 을 40/1 과 비교했던 오탐).
                #    전용 절 차례에서 '공통+전용' 으로 한 번에 검사한다.
                overridden = {"/" + k.lstrip("/") for k in doc
                              if k != "/**" and isinstance(doc[k], dict)}
                targets = [n for n in targets
                           if not any(n == o or n.endswith(o) for o in overridden)]
            else:
                base = "/" + sect.lstrip("/")
                targets = [n for n in nodes if n == base or n.endswith(base)]
                if not targets:
                    n_absent += 1
                    print(f"⚪ {f} [{sect}] → 노드가 안 떠 있음 ({len(own)}개 미검사)")
                    continue
                own_keys = set(own)          # 전용 절의 키만 미선언으로 센다
                merged = {**common, **own}   # 공통 위에 전용을 덮는다

            for node in targets:
                p = live.params(node)
                if not p:
                    n_nodump += 1
                    print(f"⚠️ {node}: 파라미터를 못 읽었다 — 판정 보류(도구 문제일 수 있음)")
                    continue
                bad_u = [k for k in merged if k not in p and k in own_keys]
                # ℹ️ yaml 의 **빈 배열**은 ROS2 가 타입을 못 정해 'not set' 이 된다.
                #    코드가 `or []` 로 받으면 동작은 같다 — 버그가 아니므로 따로 센다.
                #    (north_goal_angle 의 geofence_polygon 이 그 경우다)
                note = [k for k in merged
                        if k in p and merged[k] == []
                        and str(p[k]).strip() == "# Parameter not set"]
                bad_m = [(k, merged[k], p[k]) for k in merged
                         if k in p and k not in note and not same(merged[k], p[k])]
                if bad_u or bad_m or note:
                    print(f"\n▶ {node}   ({f} [{sect}])")
                for k in note:
                    n_note += 1
                    print(f"   ℹ️ 빈 배열  {k}  → ROS2 가 타입을 못 정해 'not set'. "
                          f"코드가 `or []` 로 받으면 동작 동일")
                for k in bad_u:
                    n_undecl += 1
                    print(f"   🚨 미선언  {k}")
                for k, exp, got in bad_m:
                    n_mismatch += 1
                    print(f"   🚨 불일치  {k}   yaml={exp!r}  실제={got!r}")

    print("\n" + "=" * 66)
    print(f"  미선언 {n_undecl} / 불일치 {n_mismatch} / 참고 {n_note} / 노드없음 {n_absent} / 판정보류 {n_nodump}")
    print("=" * 66)
    if n_mismatch:
        print("  🚨 불일치 = **yaml 이 안 실렸다**. launch 의 parameters= 를 확인할 것.")
    if n_nodump:
        print("  ⚠️ 판정보류가 있으면 다시 돌려볼 것 — 부하가 높으면 dump 가 실패한다.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
