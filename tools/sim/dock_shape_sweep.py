#!/usr/bin/env python3
"""tools/sim/dock_shape_sweep.py — 작은 표식의 픽셀 외곽선이 classify_shape 에서 어떻게 분류되나.

도형(사각/원/삼각)을 크기 12~90 px, 회전 0/7/15/30/45° 로 그리고(1px 블러 + medianBlur 3, dock 노드와 같은 마스크 처리),
dock 노드와 같은 approxPolyDP(0.0315) → classify_shape 를 돌려 원형도 임계별 정답률을 낸다.
조명 무관한 순수 픽셀 기하 시험이다. --features 는 test_dock_logic.py 에 박을 특징값을 찍는다.

    python3 tools/sim/dock_shape_sweep.py              # 임계 0.82/0.85/0.87/0.90 정답률 표
    python3 tools/sim/dock_shape_sweep.py --features   # (도형, 크기, 회전, v, area, perimeter, w, h)
근거 문서: docs/문제와작업/도킹비전_순광역광_합성시험_20261108.md
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                'src', 'color_shape_detector'))
from color_shape_detector.dock_logic import classify_shape  # noqa: E402

SIZES = (12, 14, 16, 20, 25, 30, 40, 60, 90)
ANGLES = (0, 7, 15, 30, 45)
SHAPES = ('Square', 'Circle', 'Triangle')
EPS = 0.0315


def mask(shape, s, ang, jitter=(0.0, 0.0)):
    m = np.zeros((200, 200), np.uint8)
    c = 100
    if shape == 'Square':
        cv2.rectangle(m, (int(c - s / 2), int(c - s / 2)), (int(c + s / 2), int(c + s / 2)), 255, -1)
    elif shape == 'Circle':
        cv2.circle(m, (c, c), int(s / 2), 255, -1)
    else:
        cv2.fillPoly(m, [np.array([[c, c - s / 2], [c - s / 2, c + s / 2], [c + s / 2, c + s / 2]],
                                  np.int32)], 255)
    rot = cv2.getRotationMatrix2D((c + jitter[0], c + jitter[1]), ang, 1.0)
    m = cv2.warpAffine(m, rot, (200, 200))
    m = (cv2.GaussianBlur(m, (3, 3), 0) > 127).astype(np.uint8) * 255
    return cv2.medianBlur(m, 3)


def features(m):
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cs:
        return None
    c = max(cs, key=cv2.contourArea)
    a = cv2.contourArea(c)
    p = cv2.arcLength(c, True)
    ap = cv2.approxPolyDP(c, EPS * p, True)
    x, y, w, h = cv2.boundingRect(ap)
    return len(ap), float(a), float(p), int(w), int(h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--features', action='store_true')
    args = ap.parse_args()
    if args.features:
        print(f"# cv2 {cv2.__version__}, eps {EPS}, 지터 없음")
        for sh in ('Square', 'Circle'):
            for s in (12, 16, 20, 25, 30):
                for ang in (0, 15, 45):
                    f = features(mask(sh, s, ang))
                    print(f"    ('{sh}', {s}, {ang}, {f[0]}, {f[1]:.1f}, {f[2]:.3f}, {f[3]}, {f[4]}),")
        return
    rng = np.random.default_rng(0)
    print("원형도 임계별 정답률(%) — 열: 크기 px", SIZES)
    for cmin in (0.82, 0.85, 0.87, 0.90):
        parts = []
        for sh in SHAPES:
            row = []
            for s in SIZES:
                ok = []
                for ang in ANGLES:
                    for _ in range(4):
                        f = features(mask(sh, s, ang, tuple(rng.uniform(-.5, .5, 2))))
                        ok.append(f is not None and classify_shape(*f, circle_circularity_min=cmin) == sh)
                row.append(f"{int(np.mean(ok) * 100):3d}")
            parts.append(f"{sh[:3]} " + " ".join(row))
        print(f"cmin {cmin:.2f}: " + " | ".join(parts))


if __name__ == '__main__':
    main()
