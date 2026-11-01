#!/usr/bin/env python3
"""영상 처리 단계 그림 — 보고서 Ⅲ-2.4. 실제 OAK 화면 한 장에 검출기와 같은 처리를 단계별로 다시 적용한다.

입력: docs/전달용/OAK시험_20261031/사진/04_도킹판정_예비_원.png (OAK-1 PoE 640×360, 도킹 판정 도구 화면.
      하늘색 가로선 두 줄 = 검출 높이 띠 y_zone 0.15·0.55, 글자·초록 윤곽은 그 도구가 그린 것)
처리: src/color_shape_detector 의 실제 함수와 config/vision.yaml 값을 그대로 쓴다.
      HSV 변환 → hsv.red 범위 마스크 → medianBlur 3 → 외곽 윤곽 → approxPolyDP(ε 0.0315·둘레)
      → classify_shape(꼭짓점 수 + 원형도) → 높이 띠 필터 → angle_from_pixel(hfov 63.9°, OAK 실측)
재현: python3 tools/vision_pipeline_figure.py  → docs/전달용/배치도/vision_pipeline.png
"""
import os, sys, re
import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"; matplotlib.rcParams["axes.unicode_minus"] = False
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
PKG = os.path.join(ROOT, "src", "color_shape_detector")
sys.path.insert(0, PKG)
from color_shape_detector.dock_logic import classify_shape, circularity   # noqa: E402
from color_shape_detector.vision_geom import angle_from_pixel            # noqa: E402

IMG = os.path.join(ROOT, "docs", "전달용", "OAK시험_20261031", "사진", "04_도킹판정_예비_원.png")
OUT = os.path.join(ROOT, "docs", "전달용", "배치도", "vision_pipeline.png")
HFOV_OAK = 63.9


def yaml_section(text, name):
    """vision.yaml 에서 한 노드 절의 'key: value' 만 읽는다(주석 제거). 공통 절은 '/**'."""
    out, cur = {}, None
    for line in text.splitlines():
        s = line.split("#")[0].rstrip()
        if not s.strip(): continue
        if not line.startswith(" ") and s.endswith(":"):
            cur = s[:-1].strip(); continue
        if cur == name and ":" in s and "ros__parameters" not in s:
            k, v = s.strip().split(":", 1); out[k.strip()] = v.strip()
    return out


Y = open(os.path.join(PKG, "config", "vision.yaml"), encoding="utf-8").read()
common, dock = yaml_section(Y, "/**"), yaml_section(Y, "image_subscriber_dock")
red = [int(v) for v in re.findall(r"-?\d+", common["hsv.red"])]
RANGES = [(red[i:i + 3], red[i + 3:i + 6]) for i in range(0, len(red), 6)]
P = {k: float(dock[k]) for k in ("approx_epsilon", "min_area_px", "square_extent_min", "square_aspect_lo", "square_aspect_hi",
                                 "circle_circularity_min", "square_circularity_max", "y_zone_lo", "y_zone_hi")}

bgr = cv2.imread(IMG); H, W = bgr.shape[:2]
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
mask = np.zeros((H, W), np.uint8)
for lo, hi in RANGES:
    mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
mask = cv2.medianBlur(mask, 3)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

cands = []
for c in contours:
    area = cv2.contourArea(c)
    if area < P["min_area_px"]:
        continue
    per = cv2.arcLength(c, True)
    ap = cv2.approxPolyDP(c, P["approx_epsilon"] * per, True)
    x, y, w, h = cv2.boundingRect(ap)
    shape = classify_shape(len(ap), area, per, w, h, min_area=P["min_area_px"],
                           square_extent_min=P["square_extent_min"], square_aspect_lo=P["square_aspect_lo"],
                           square_aspect_hi=P["square_aspect_hi"], circle_circularity_min=P["circle_circularity_min"],
                           square_circularity_max=P["square_circularity_max"])
    v = ap.reshape(-1, 2); vx, vy = int(v[:, 0].mean()), int(v[:, 1].mean())
    in_band = H * P["y_zone_lo"] <= vy <= H * P["y_zone_hi"]
    cands.append(dict(ap=ap, area=area, circ=circularity(area, per), n=len(ap), shape=shape, vx=vx, vy=vy, ok=bool(shape) and in_band))

rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
fig, axs = plt.subplots(2, 2, figsize=(12, 7.2))
y0, y1 = H * P["y_zone_lo"], H * P["y_zone_hi"]

ax = axs[0, 0]; ax.imshow(rgb); ax.set_title("(a) 입력 — OAK-1 PoE 640×360 실제 화면", fontsize=11)
ax = axs[0, 1]; ax.imshow(mask, cmap="gray"); ax.set_title("(b) HSV 빨강 범위 마스크 + 중앙값 필터(3×3)", fontsize=11)
ax = axs[1, 0]; ax.imshow(rgb, alpha=0.45); ax.set_title("(c) 윤곽 근사 → 형상 판정 → 높이 띠 필터", fontsize=11)
ax.axhspan(y0, y1, color="#3ab0ff", alpha=0.12); ax.axhline(y0, color="#3ab0ff", lw=1); ax.axhline(y1, color="#3ab0ff", lw=1)
for c in cands:
    p = np.vstack([c["ap"].reshape(-1, 2), c["ap"].reshape(-1, 2)[:1]])
    col = "#1a9c3a" if c["ok"] else "#d03030"
    ax.plot(p[:, 0], p[:, 1], color=col, lw=2)
    why = "채택" if c["ok"] else ("형상 불명" if not c["shape"] else "높이 띠 밖")
    ax.text(c["vx"] + 40, c["vy"] + 30 if c["vy"] < H / 2 else c["vy"] - 40,
            f"{c['shape'] or '?'} · 꼭짓점 {c['n']} · 원형도 {c['circ']:.2f}\n→ {why}", fontsize=7.5, color=col,
            bbox=dict(fc="white", ec="none", alpha=0.75, pad=1))
ax = axs[1, 1]; ax.imshow(rgb); ax.set_title("(d) 방위각 변환 θ = -atan((x - W/2) / fx)", fontsize=11)
ax.axvline(W / 2, color="#ffcc00", lw=1, ls="--")
for c in [c for c in cands if c["ok"]]:
    th = angle_from_pixel(c["vx"], W, HFOV_OAK)
    ax.plot([W / 2, c["vx"]], [c["vy"], c["vy"]], color="#1a9c3a", lw=1.5)
    ax.plot(c["vx"], c["vy"], "o", color="#1a9c3a")
    ax.text(c["vx"] + 10, c["vy"] + 28, f"{c['shape']}  θ = {th:+.1f}°\n(HFOV 63.9°, 왼쪽 +)", fontsize=9, color="#1a9c3a",
            bbox=dict(fc="white", ec="none", alpha=0.8, pad=1))
for a in axs.flat:
    a.set_xticks([]); a.set_yticks([])
fig.tight_layout()
fig.savefig(OUT, dpi=130)
print(OUT, [(c["shape"], c["n"], round(c["circ"], 2), c["vy"], c["ok"]) for c in cands])
