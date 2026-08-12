#!/usr/bin/env python3
"""vision_probe — 색 검출을 **수치로** 검증하고, 실패한 순간만 사진으로 남긴다.

    source install/setup.bash
    python3 tools/vision_probe.py --sec 60 --colors red,green

왜 이렇게 만들었나
  · 영상을 계속 보는 건 비싸고(픽셀당 비용) 정보도 적다 — 물체가 가만히 있으면
    1초 전 프레임과 지금 프레임이 거의 같다.
  · 튜닝에서 알고 싶은 건 "잘 될 때" 가 아니라 **"언제 실패하나"** 다.
    그래서 분석은 전 프레임 계속 하고(비용 0), **사진은 사건이 있을 때만** 남긴다.
  · 마지막에 격자 한 장으로 묶는다 — 타일을 줄이면 9순간을 한 장 값에 볼 수 있다.

사건 = 검출 시작 / 검출 놓침 / 각도 5° 이상 튐 / **덩어리 개수 변화**
  🚨 덩어리 개수 변화가 제일 중요하다. 그게 '진짜 물체 말고 뭐가 더 잡히는지' 를 드러낸다.

⚠️ HSV·화각은 `config/vision.yaml` 을 그대로 읽는다. 검출기와 **같은 값·같은 수식**이라
   여기서 본 각도가 곧 `/red_angle` 값이다. 따로 계산하면 또 두 벌이 된다.
"""

import argparse
import csv
import datetime
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from PIL import Image as PILImage, ImageDraw, ImageFont
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from color_shape_detector import hsv_ranges
from color_shape_detector.vision_geom import angle_from_pixel

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=1)

ANGLE_JUMP_DEG = 5.0     # 이보다 튀면 사건
MAX_SHOTS = 9            # 격자 3x3
MIN_SHOT_GAP_S = 1.5     # 사진 사이 최소 간격
BLOB_DEBOUNCE = 4        # 덩어리 개수는 이만큼 연속 유지돼야 '변했다' 로 친다

# 🚨 사건 우선순위. 칸이 차면 **낮은 것을 밀어내고** 높은 것을 남긴다.
#    첫 시험에서 '덩어리 2↔3' 깜빡임이 0.6초 만에 9칸을 다 먹어치웠다 —
#    정작 놓침·각도 튐 같은 진짜 사건은 하나도 못 담았다.
PRIO = {"lost": 3, "angle": 2, "found": 2, "blobs": 1}
TILE = (320, 240)
DRAW = {"red": (0, 0, 255), "green": (0, 255, 0), "white": (255, 255, 255),
        "orange": (0, 140, 255), "yellow": (0, 255, 255), "blue": (255, 0, 0)}

# 🚨 OpenCV putText 는 한글을 못 그린다 — 전부 '???' 가 된다(실제로 그렇게 나왔다).
#    격자 라벨만 PIL 로 그린다. 폰트가 없는 환경이면 **영문 라벨로 대체**한다
#    (물음표를 남기지 않는다 — 읽을 수 없는 라벨은 없는 것만 못하다).
_FONT_CANDIDATES = [
    ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 0),
    ("/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc", 0),
    ("/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 0),
]


def _load_font(size=15):
    for path, idx in _FONT_CANDIDATES:
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size, index=idx)
            except Exception:                    # noqa: BLE001
                continue
    return None


def draw_labels(sheet_bgr, items, font):
    """격자에 라벨을 그린다. font=None 이면 호출자가 영문 라벨을 넘긴 상태다."""
    pil = PILImage.fromarray(cv2.cvtColor(sheet_bgr, cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(pil)
    for text, (x, y) in items:
        d.text((x, y), text, font=font, fill=(255, 255, 0))
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


def circ_hue_stats(h_arr):
    """색상(H)의 중앙값·5/95% — **원형통계**로 낸다.

    🚨 빨강은 색상환 0 을 넘어간다(H 178 과 2 는 이웃이다).
       일반 백분위를 쓰면 p5=0, p95=179 처럼 **색상환 전체**가 나와 무의미하다
       (실측에서 실제로 그렇게 나왔다).
       yaw_mux 가 방위 평균에 쓴 것과 같은 방법이다: 단위벡터로 평균을 구해
       그쪽으로 회전시킨 뒤 백분위를 재고 되돌린다.
    """
    h = np.asarray(h_arr, dtype=float)
    if h.size == 0:
        return None, None, None
    th = h * (2.0 * np.pi / 180.0)          # 0~180 → 0~2π
    mean_th = np.arctan2(np.sin(th).mean(), np.cos(th).mean())
    d = np.angle(np.exp(1j * (th - mean_th)))          # 평균 기준 -π~π
    lo, med, hi = np.percentile(d, [5, 50, 95])
    to_h = lambda x: int(round(((x + mean_th) % (2 * np.pi)) * 180.0 / (2 * np.pi))) % 180
    return to_h(med), to_h(lo), to_h(hi)


def load_vision_yaml():
    """vision.yaml 에서 hsv.*·hfov_deg·min_area 를 읽는다. 못 읽으면 기본값."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('color_shape_detector'),
                         'config', 'vision.yaml')
        with open(p, encoding='utf-8') as f:
            doc = yaml.safe_load(f)
        params = doc.get('/**', {}).get('ros__parameters', {})
        return params, p
    except Exception as e:                       # noqa: BLE001
        print(f"⚠️ vision.yaml 을 못 읽었다({e}) → 기본값 사용")
        return {}, "(기본값)"


class Probe(Node):

    def __init__(self, colors, seconds, min_area, log_csv=None, log_every=5.0):
        super().__init__('vision_probe')
        params, path = load_vision_yaml()
        self.hfov = float(params.get('hfov_deg', 80.32))
        topic = params.get('image_topic', '/camera/camera/color/image_raw')
        self.ranges = hsv_ranges.load(
            lambda n, d: params.get(n, d), hsv_ranges.VALID_COLORS,
            on_error=lambda m: print(m))

        print(f"설정   : {path}")
        print(f"화각   : {self.hfov}°   토픽: {topic}")
        print(f"대상색 : {', '.join(colors)}   최소면적 {min_area}px   {seconds}초\n")

        self.colors = colors
        self.min_area = min_area
        self.deadline = time.monotonic() + seconds
        self.br = CvBridge()
        self.frames = 0
        self.t0 = None

        # 색별 누적
        self.stat = {c: {"hit": 0, "angles": [], "areas": [], "blobs": [],
                         "spread": [], "bias": []} for c in colors}
        self.prev = {c: {"seen": False, "angle": None, "blobs": 0,
                         "pend_blobs": None, "pend_n": 0} for c in colors}
        self.last_shot_t = -1e9
        self.event_counts = {}   # 종류별 총 발생 수(사진을 못 남겨도 센다)
        self.shots = []          # (우선순위, 한글라벨, 영문라벨, 이미지)

        # ── 연습기간 실측 로깅 ──────────────────────────────────
        # 🚨 사람이 기억해야 하는 절차는 결국 잊힌다(blackbox 와 같은 발상).
        #    연습 나갈 때 그냥 켜두면 시간대별 HSV 가 CSV 로 쌓인다.
        #    ⚠️ 부표만이 아니라 **화면 전체 밝기(frame_v_med)와 마스크 비율**도 남긴다 —
        #       "이 범위가 안전한가" 는 부표 색이 아니라 **배경과 갈리는가** 로 정해진다.
        self.log_path = log_csv
        self.log_every = log_every
        self._last_log = -1e9
        self._csv = None
        self._w = None
        if log_csv:
            new = not os.path.exists(log_csv)
            self._csv = open(log_csv, 'a', newline='', encoding='utf-8')
            self._w = csv.writer(self._csv)
            if new:
                self._w.writerow([
                    "iso_time", "elapsed_s", "color", "detected", "n_blobs",
                    "top_area", "angle_deg", "bias_deg", "mask_pct", "frame_v_med",
                    "h_med", "h_p5", "h_p95", "s_med", "s_p5", "s_p95",
                    "v_med", "v_p5", "v_p95"])
            print(f"📝 로깅: {log_csv}  ({log_every}초 간격)")

        self.create_subscription(Image, topic, self.cb, QOS)

    # ── 한 프레임 분석 ────────────────────────────────────────────
    def analyze(self, hsv, color, width):
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.ranges[color]:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = cv2.medianBlur(mask, 3)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = [c for c in cnts if cv2.contourArea(c) >= self.min_area]
        if not big:
            return None, [], mask
        top = max(big, key=cv2.contourArea)
        M = cv2.moments(top)
        if M["m00"] <= 0:
            return None, big, mask
        cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
        # 모든 덩어리의 각도 — '한 물체가 쪼개진 것' 과 '딴 게 잡힌 것' 을 가른다
        angs, wsum, asum = [], 0.0, 0.0
        for c in big:
            m2 = cv2.moments(c)
            if m2["m00"] <= 0:
                continue
            a2 = cv2.contourArea(c)
            g = angle_from_pixel(int(m2["m10"] / m2["m00"]), width, self.hfov)
            angs.append(g)
            wsum += a2
            asum += a2 * g
        wmean = asum / wsum if wsum > 0 else None
        # 검출된 '가장 큰 덩어리' 안쪽 픽셀의 HSV 분포 — 연습기간 시간대별 실측용
        mm = np.zeros(hsv.shape[:2], np.uint8)
        cv2.drawContours(mm, [top], -1, 255, -1)
        px = hsv[mm > 0]
        st = {}
        if px.size:
            # H 만 원형통계 — S·V 는 선형이라 그냥 백분위로 낸다
            st["h_med"], st["h_p5"], st["h_p95"] = circ_hue_stats(px[:, 0])
            for i, nm in ((1, "s"), (2, "v")):
                arr = px[:, i]
                st[nm + "_med"] = int(np.median(arr))
                st[nm + "_p5"] = int(np.percentile(arr, 5))
                st[nm + "_p95"] = int(np.percentile(arr, 95))
        return ((cx, cy, cv2.contourArea(top),
                 angle_from_pixel(cx, width, self.hfov), angs, wmean, st), big, mask)

    def cb(self, msg):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        if now > self.deadline:
            raise KeyboardInterrupt

        frame = self.br.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.frames += 1
        h, w = frame.shape[:2]

        events = []
        overlay = frame.copy()
        cv2.line(overlay, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)

        for c in self.colors:
            res, big, _ = self.analyze(hsv, c, w)
            st, pv = self.stat[c], self.prev[c]
            cv2.drawContours(overlay, big, -1, DRAW[c], 2)

            if res is None:
                if pv["seen"]:
                    events.append(("lost", f"{c} 놓침", f"{c} lost"))
                pv.update(seen=False, angle=None, blobs=0)
                continue

            cx, cy, area, ang, angs, wmean, hstat = res
            st["hit"] += 1
            st["angles"].append(ang)
            st["areas"].append(area)
            st["blobs"].append(len(big))
            if len(angs) > 1:
                st["spread"].append(max(angs) - min(angs))
                if wmean is not None:
                    st["bias"].append(ang - wmean)   # 최대덩어리만 쓸 때의 각도 치우침
            cv2.circle(overlay, (cx, cy), 7, DRAW[c], -1)
            cv2.putText(overlay, f"{c} {ang:+.1f}deg", (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(overlay, f"{c} {ang:+.1f}deg", (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, DRAW[c], 1)

            if not pv["seen"]:
                events.append(("found", f"{c} 검출시작 {ang:+.0f}°", f"{c} found {ang:+.0f}deg"))
            elif pv["angle"] is not None and abs(ang - pv["angle"]) >= ANGLE_JUMP_DEG:
                events.append(("angle", f"{c} 각도 {pv['angle']:+.0f}→{ang:+.0f}°",
                               f"{c} angle {pv['angle']:+.0f}->{ang:+.0f}deg"))
            # 덩어리 개수는 **연속 BLOB_DEBOUNCE 프레임** 유지돼야 변한 것으로 친다.
            # 안 그러면 경계 픽셀 하나로 2↔3 이 깜빡이며 사건이 폭주한다(실측).
            nb = len(big)
            if nb != pv["blobs"]:
                if pv["pend_blobs"] == nb:
                    pv["pend_n"] += 1
                else:
                    pv["pend_blobs"], pv["pend_n"] = nb, 1
                if pv["pend_n"] >= BLOB_DEBOUNCE:
                    events.append(("blobs", f"{c} 덩어리 {pv['blobs']}→{nb}",
                                   f"{c} blobs {pv['blobs']}->{nb}"))
                    pv["blobs"] = nb
                    pv["pend_blobs"], pv["pend_n"] = None, 0
            else:
                pv["pend_blobs"], pv["pend_n"] = None, 0
            pv.update(seen=True, angle=ang)

        if self._w is not None and now - self._last_log >= self.log_every:
            self._last_log = now
            iso = datetime.datetime.now().isoformat(timespec='seconds')
            fv = int(np.median(hsv[:, :, 2]))
            for c in self.colors:
                r2, big2, m2 = self.analyze(hsv, c, w)
                pct = 100.0 * int(m2.sum() // 255) / m2.size
                if r2 is None:
                    self._w.writerow([iso, f"{now-self.t0:.1f}", c, 0, len(big2),
                                      "", "", "", f"{pct:.3f}", fv] + [""] * 9)
                else:
                    _cx, _cy, a2, g2, ags, wm, hs = r2
                    bias = (g2 - wm) if wm is not None else ""
                    self._w.writerow([
                        iso, f"{now-self.t0:.1f}", c, 1, len(big2), int(a2),
                        f"{g2:.2f}", (f"{bias:.2f}" if bias != "" else ""),
                        f"{pct:.3f}", fv,
                        hs.get("h_med", ""), hs.get("h_p5", ""), hs.get("h_p95", ""),
                        hs.get("s_med", ""), hs.get("s_p5", ""), hs.get("s_p95", ""),
                        hs.get("v_med", ""), hs.get("v_p5", ""), hs.get("v_p95", "")])
            self._csv.flush()

        if not events:
            return
        for kind, _, _ in events:
            self.event_counts[kind] = self.event_counts.get(kind, 0) + 1

        t = now - self.t0
        prio = max(PRIO.get(k, 0) for k, _, _ in events)
        kr = f"{t:4.1f}s  " + " / ".join(e[1] for e in events)
        en = f"{t:4.1f}s  " + " / ".join(e[2] for e in events)

        # 최소 간격 — 같은 순간을 여러 장 남기지 않는다. 단 최우선(놓침)은 예외.
        if now - self.last_shot_t < MIN_SHOT_GAP_S and prio < 3:
            return

        if len(self.shots) < MAX_SHOTS:
            self.shots.append((prio, kr, en, overlay))
        else:
            worst = min(range(len(self.shots)), key=lambda i: self.shots[i][0])
            if prio <= self.shots[worst][0]:
                return                      # 남길 가치가 더 낮다 → 버린다
            self.shots[worst] = (prio, kr, en, overlay)
        self.last_shot_t = now
        print(f"  📸 {kr}")

    # ── 결과 ─────────────────────────────────────────────────────
    def report(self, out_dir):
        print("\n" + "=" * 66)
        print(f"  프레임 {self.frames}장 분석")
        print("=" * 66)
        for c in self.colors:
            s = self.stat[c]
            if not s["hit"]:
                print(f"  [{c}] 한 번도 검출 안 됨 ({self.frames}프레임 전부)")
                continue
            a = np.array(s["angles"])
            ar = np.array(s["areas"])
            bl = np.array(s["blobs"])
            rate = 100.0 * s["hit"] / max(self.frames, 1)
            print(f"  [{c}] 검출 {s['hit']}/{self.frames} ({rate:.0f}%)")
            print(f"        각도 {a.min():+.1f}° ~ {a.max():+.1f}°   (폭 {a.max()-a.min():.1f}°)")
            print(f"        면적 {int(ar.min())} ~ {int(ar.max())}px")
            uniq, cnt = np.unique(bl, return_counts=True)
            dist = "  ".join(f"{u}개:{100*n//len(bl)}%" for u, n in zip(uniq, cnt))
            print(f"        덩어리 분포 {dist}")
            if bl.min() > 1 and s["spread"]:
                sp = np.array(s["spread"])
                print(f"        덩어리 각도 퍼짐 중앙값 {np.median(sp):.1f}°  (최대 {sp.max():.1f}°)")
                if s["bias"]:
                    bi = np.abs(np.array(s["bias"]))
                    print(f"        🎯 최대덩어리만 쓸 때 각도 치우침 "
                          f"중앙값 {np.median(bi):.2f}°  최대 {bi.max():.2f}°")
                    print(f"           (전체 면적 무게중심 대비. 이게 실제 조향 오차다)")
                # 🚨 원인을 **단정하지 않는다.** 두 번 틀렸다:
                #    ① 처음엔 "배경에 같은 색이 있다" → 실제론 소화기가 노란 띠로 쪼개진 것
                #    ② 다음엔 각도 퍼짐 6.7° 를 보고 "다른 물체다" → 실제론 선풍기 반대쪽 날개
                #    색 덩어리만으로 '한 물체냐' 를 가릴 수 없다. 숫자만 내고 판단은 사람이 한다.
                print(f"        ℹ️ 덩어리가 여럿인 이유는 둘 중 하나다 — "
                      f"한 물체가 쪼개졌거나(띠·글씨·허브), 딴 게 같이 잡혔거나.")
                print(f"           격자 사진으로 확인할 것. 퍼짐이 커도 같은 물체일 수 있다"
                      f"(날개처럼 넓게 벌어진 형상).")
        if self.event_counts:
            tot = "  ".join(f"{k}:{v}" for k, v in sorted(self.event_counts.items()))
            print(f"  사건 총계 {tot}   (사진은 우선순위·간격으로 골라 최대 {MAX_SHOTS}장)")
        print("=" * 66)

        if not self.shots:
            print("  (사건 없음 — 사진 없음)")
            return None
        cols = 3
        rows = (len(self.shots) + cols - 1) // cols
        sheet = np.zeros((rows * (TILE[1] + 26), cols * TILE[0], 3), np.uint8)
        font = _load_font(14)
        items = []
        for i, (_prio, kr, en, img) in enumerate(self.shots):
            r, cc = divmod(i, cols)
            tile = cv2.resize(img, TILE)
            y0 = r * (TILE[1] + 26)
            sheet[y0:y0 + TILE[1], cc * TILE[0]:(cc + 1) * TILE[0]] = tile
            # 폰트가 있으면 한글, 없으면 영문 — 어느 쪽이든 **읽을 수 있는** 라벨을 남긴다
            items.append(((kr if font else en)[:40], (cc * TILE[0] + 4, y0 + TILE[1] + 5)))
        if font:
            sheet = draw_labels(sheet, items, font)
        else:
            print("  ⚠️ 한글 폰트 없음 → 영문 라벨로 그린다")
            for text, (x, y) in items:
                cv2.putText(sheet, text, (x, y + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
        path = os.path.join(out_dir, "vision_probe_sheet.png")
        cv2.imwrite(path, sheet)
        print(f"  격자 저장: {path}  ({len(self.shots)}순간)")
        return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sec', type=float, default=60.0)
    ap.add_argument('--colors', default='red,green')
    ap.add_argument('--min-area', type=float, default=40.0)
    ap.add_argument('--out', default='/tmp')
    ap.add_argument('--log-csv', default=None,
                    help='연습기간 실측 로그를 이어붙일 CSV 경로(없으면 로깅 안 함)')
    ap.add_argument('--log-every', type=float, default=5.0)
    a = ap.parse_args()

    rclpy.init()
    node = Probe([c.strip() for c in a.colors.split(',') if c.strip()],
                 a.sec, a.min_area, a.log_csv, a.log_every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report(a.out)
        if node._csv:
            node._csv.close()
            print(f"  📝 로그 저장: {node.log_path}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
