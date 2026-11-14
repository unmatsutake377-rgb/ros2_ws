#!/usr/bin/env python3
"""토너먼트 출발 위치에서 IMU 방위 오프셋(mount_offset_deg) 잡기 — 물 위 직진(§9-0g) 대신 '겨누기'.

원리: 1번배 IMU 는 USB 전원 들어온 순간 뱃머리 = 0° 인 상대각(/imu/yaw_raw)이다.
      yaw_mux 는 /imu/yaw = (invert_yaw 면 −raw, 아니면 raw) + mount_offset_deg.
      출발 위치에서 뱃머리를 레인 결승 WP(T1) 쪽으로 똑바로 겨눠 세우면 참 방위 = RTK 현재 위치 → T1 방위.
      → mount_offset_deg = 그 방위 − (invert 적용한) raw.

쓰는 법 (대회 launch 가 떠 있고 RC MANUAL, 배가 멈춘 상태):
  1) tools/ssf_tournament.sh A 1            ← 다른 창. IMU USB 는 이후 경기 끝까지 뽑지 않는다
  2) 뱃머리를 결승 쪽 T1 에 똑바로 겨누고 멈춤(밀리지 않게 잡는다)
  3) python3 tools/ssf_imu_aim.py A 1       ← 5 s 평균 → 오프셋 계산 → y 면 ssf_heading_A.yaml 에 기록
  4) 1) 창에서 Ctrl+C 한 번 → tools/ssf_tournament.sh A 1 다시(USB 그대로) → /imu/yaw 가 표시된 목표 ±2° 인지
  옵션: --target-bearing 234.0  (T1 대신 아는 방위로 겨눈 경우, 진북 기준 도)
        --seconds 5  --dry(파일 안 씀)

정확도: 겨눈 선이 T1 에서 옆으로 e m 빗나가면 오차 ≈ atan(e / 거리). 33 m 에서 1 m = 1.7°.
        RTK 위치 오차(cm)는 무시할 만하다. 정지 중 raw 흔들림 표준편차가 1° 넘으면 거부(배가 돌고 있음).
⚠️ 이 값은 IMU 전원마다 다르다 → **커밋하지 않는다**(배 노트북 로컬, §9-0g 와 같음).
   USB 재연결·허브 끊김·재부팅 뒤엔 다시. 시간이 지나면 자이로가 조금씩 틀어지니 경기 직전 한 번 더.
"""
import argparse
import math
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LANE_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,15}$')
OFFSET_RE = re.compile(r'^(\s*mount_offset_deg:\s*)([-+0-9.eE]+)(.*)$', re.M)
INVERT_RE = re.compile(r'^\s*invert_yaw:\s*(true|false)\b', re.M | re.I)
MAX_STD_DEG = 1.0
MIN_DIST_M = 10.0


def wrap360(a):
    return a % 360.0


def bearing_deg(lat1, lon1, lat2, lon2):
    """1 → 2 진북 기준 방위(도). 수십 m 라 구면 초기 방위 공식 그대로."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return wrap360(math.degrees(math.atan2(y, x)))


def dist_m(lat1, lon1, lat2, lon2):
    k = 111320.0 * math.cos(math.radians((lat1 + lat2) / 2.0))
    return math.hypot((lon2 - lon1) * k, (lat2 - lat1) * 110574.0)


def circ_mean_std(angles_deg):
    """원형 평균(0~360)과 원형 표준편차(도)."""
    if not angles_deg:
        raise ValueError('표본 없음')
    c = sum(math.cos(math.radians(a)) for a in angles_deg) / len(angles_deg)
    s = sum(math.sin(math.radians(a)) for a in angles_deg) / len(angles_deg)
    r = min(1.0, math.hypot(c, s))
    std = math.degrees(math.sqrt(-2.0 * math.log(r))) if r > 0 else 180.0
    return wrap360(math.degrees(math.atan2(s, c))), std


def mount_offset(target_deg, raw_deg, invert):
    """yaw_mux 식 /imu/yaw = wrap360((−raw if invert else raw) + offset) 를 target 으로 만드는 offset."""
    y = -raw_deg if invert else raw_deg
    return round(wrap360(target_deg - y), 1)


def read_invert(text):
    m = INVERT_RE.search(text)
    if not m:
        raise ValueError('invert_yaw 줄을 못 찾음')
    return m.group(1).lower() == 'true'


def set_offset(text, offset):
    """mount_offset_deg 값만 바꾼다(주석 유지). 늘 소수점 — 정수면 yaw_mux 가 즉사한다."""
    new, n = OFFSET_RE.subn(lambda m: f'{m.group(1)}{float(offset):.1f}{m.group(3)}', text, count=1)
    if n != 1:
        raise ValueError('mount_offset_deg 줄을 못 찾음')
    return new


def lane_file(boat, lane, home=None):
    """ssf_tournament.sh 와 같은 순서: 배 자기 녹화본 → 공유 레인."""
    home = home or os.path.expanduser('~')
    own = os.path.join(home, 'ssf_logs', f'waypoints_tournament_{boat}_{lane}.yaml')
    shared = os.path.join(REPO, 'tools', 'tournament_lanes', f'waypoints_tournament_{lane}.yaml')
    for p in (own, shared):
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(f'레인 녹화본 없음: {own} (공유 {shared} 도 없음)')


def last_wp(path):
    import yaml
    with open(path, encoding='utf-8') as f:
        wps = (yaml.safe_load(f) or {}).get('waypoints') or []
    if len(wps) < 2:
        raise ValueError(f'웨이포인트 {len(wps)}개 — 2개 이상 필요 ({path})')
    return float(wps[-1]['lat']), float(wps[-1]['lon'])


def collect(seconds):
    """/imu/yaw_raw·/imu/yaw·/ublox_gps_node/fix 를 seconds 동안 모은다(대회 launch 가 떠 있어야 한다)."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data as qos   # BEST_EFFORT — RELIABLE 발행자와도 맞는다(읽기 전용)
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import Float64

    rclpy.init()
    node = rclpy.create_node('ssf_imu_aim')
    raw, yaw, fixes = [], [], []
    node.create_subscription(Float64, '/imu/yaw_raw', lambda m: raw.append(m.data), qos)
    node.create_subscription(Float64, '/imu/yaw', lambda m: yaw.append(m.data), qos)

    def fix_cb(m):
        if m.status.status >= 0 and math.isfinite(m.latitude) and (m.latitude, m.longitude) != (0.0, 0.0):
            fixes.append((m.latitude, m.longitude, m.position_covariance[0]))
    node.create_subscription(NavSatFix, '/ublox_gps_node/fix', fix_cb, qos)
    end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return raw, yaw, fixes


def main(argv=None):
    ap = argparse.ArgumentParser(description='출발 위치 겨누기로 IMU mount_offset_deg 잡기')
    ap.add_argument('boat', type=str.upper, choices=['A', 'B'])
    ap.add_argument('lane')
    ap.add_argument('--target-bearing', type=float, default=None, help='T1 대신 아는 진북 방위(도)로 겨눴을 때')
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--dry', action='store_true', help='계산만, yaml 안 씀')
    a = ap.parse_args(argv)
    if not LANE_RE.match(a.lane):
        sys.exit(f"❌ 레인 이름 형식: '{a.lane}'")
    marker = os.path.expanduser('~/.ssf_boat')
    if os.path.isfile(marker):
        mine = open(marker, encoding='utf-8').read().strip().upper()
        if mine != a.boat:
            sys.exit(f'❌ 이 노트북은 {mine} 번배(~/.ssf_boat)인데 {a.boat} 로 하려 했다')
    yml = os.path.join(REPO, 'src', 'ssf_heading', 'config', f'ssf_heading_{a.boat}.yaml')
    text = open(yml, encoding='utf-8').read()
    invert = read_invert(text)

    print(f'… {a.seconds:.0f} s 모으는 중 — 배를 움직이지 말 것(대회 launch 가 떠 있어야 한다)')
    raw, yaw, fixes = collect(a.seconds)
    if len(raw) < 5:
        sys.exit(f'❌ /imu/yaw_raw 표본 {len(raw)}개 — 대회 launch(iahrs 드라이버)가 떠 있나, ROS_LOCALHOST_ONLY 같나')
    raw_mean, raw_std = circ_mean_std(raw)
    if raw_std > MAX_STD_DEG:
        sys.exit(f'❌ 정지 중 raw 흔들림 {raw_std:.2f}° > {MAX_STD_DEG}° — 배가 돌고 있다. 잡고 다시')

    if a.target_bearing is not None:
        target, how = wrap360(a.target_bearing), f'지정 방위 {wrap360(a.target_bearing):.1f}°'
    else:
        if not fixes:
            sys.exit('❌ GPS fix 없음 — RTK 확인(또는 --target-bearing)')
        lat = sum(f[0] for f in fixes) / len(fixes)
        lon = sum(f[1] for f in fixes) / len(fixes)
        sd = math.sqrt(max(0.0, sum(f[2] for f in fixes) / len(fixes)))
        lp = lane_file(a.boat, a.lane)
        tlat, tlon = last_wp(lp)
        d = dist_m(lat, lon, tlat, tlon)
        if d < MIN_DIST_M:
            sys.exit(f'❌ 결승 WP 까지 {d:.1f} m — {MIN_DIST_M:.0f} m 넘게 떨어진 출발 위치에서 겨눠야 정확하다')
        target = bearing_deg(lat, lon, tlat, tlon)
        how = (f'RTK 위치({lat:.7f}, {lon:.7f}, 표준편차≈{sd:.2f} m) → 결승 WP {lp} '
               f'방위 {target:.1f}°, 거리 {d:.1f} m (1 m 빗나가면 {math.degrees(math.atan(1.0 / d)):.1f}°)')

    off = mount_offset(target, raw_mean, invert)
    cur_off = OFFSET_RE.search(text)
    print(f'✓ 목표 방위: {how}')
    print(f'✓ /imu/yaw_raw 평균 {raw_mean:.1f}° (흔들림 {raw_std:.2f}°, {len(raw)}표본), invert_yaw={invert}')
    if yaw:
        print(f'  지금 /imu/yaw {circ_mean_std(yaw)[0]:.1f}° (지금 mount_offset_deg {cur_off.group(2) if cur_off else "?"})')
    print(f'▶ 새 mount_offset_deg = {off:.1f}  → 재시작 뒤 /imu/yaw ≈ {target:.1f}° 이어야 한다')
    if a.dry:
        print('(--dry — 파일 안 씀)')
        return 0
    if sys.stdin.isatty():
        ans = input(f'  {yml} 에 쓸까? [y/N] ').strip().lower()
        if ans != 'y':
            print('중단 — 안 씀')
            return 1
    else:
        sys.exit('❌ 대화형 터미널이 아니라 확인을 못 받는다 — 터미널에서 직접 실행(또는 --dry)')
    with open(yml, 'w', encoding='utf-8') as f:
        f.write(set_offset(text, off))
    inst = os.path.join(REPO, 'install', 'ssf_heading', 'share', 'ssf_heading', 'config', f'ssf_heading_{a.boat}.yaml')
    if os.path.isfile(inst) and f'mount_offset_deg: {off:.1f}' not in open(inst, encoding='utf-8').read():
        print(f'⚠️  설치본 {inst} 에 새 값이 안 보인다(symlink-install 아님?) → colcon build --packages-select ssf_heading')
    print('✓ 썼다 — 대회 launch 창 Ctrl+C 한 번 → 다시 띄움(IMU USB 그대로). 커밋하지 않는다(전원마다 다름).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
