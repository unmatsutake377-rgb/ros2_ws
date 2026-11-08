"""tools/sim/dock_sun_stress.py — 도킹 표식 순광/역광 합성 스트레스 시험 — 실제 dock 검출 파이프라인(복제) + vision.yaml HSV.
가정 모델이다. 실측 아님."""
import sys, numpy as np, cv2
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src', 'color_shape_detector'))
from color_shape_detector.dock_logic import classify_shape
from color_shape_detector import hsv_ranges
from color_shape_detector.vision_geom import angle_from_pixel

import yaml  # noqa: E402

# 파라미터는 노드와 같은 출처(config/vision.yaml: `/**` 공통 절 위에 image_subscriber_dock 절)에서 읽는다.
_VY = yaml.safe_load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src',
                                       'color_shape_detector', 'config', 'vision.yaml'), encoding='utf-8'))
_DP = dict(_VY['/**']['ros__parameters'])
_DP.update(_VY['image_subscriber_dock']['ros__parameters'])
R = {c: hsv_ranges.parse_flat(_DP[hsv_ranges.param_name(c)], c) for c in hsv_ranges.VALID_COLORS}
P = {k: float(_DP[k]) for k in ('approx_epsilon', 'min_area_px', 'square_extent_min', 'square_aspect_lo',
                                'square_aspect_hi', 'circle_circularity_min', 'square_circularity_max',
                                'y_zone_lo', 'y_zone_hi')}
W, H = 640, 360

def lin2s(x):
    x = np.clip(x, 0, 1)
    return np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1 / 2.4) - 0.055)
def s2lin(x):
    return np.where(x <= 0.04045, x / 12.92, ((x + 0.055) / 1.055) ** 2.4)

# 반사율(선형 RGB). red 는 10-31 OAK 실내 사진 실측(sRGB 92,13,9 / 벽 168)에서 역산, 나머지는 일반 도료 가정.
wall = s2lin(168 / 255)
red = np.array([s2lin(92/255), s2lin(13/255), s2lin(9/255)]) / wall * 0.8
REFL = {'red': red, 'green': np.array([.03, .20, .05]), 'blue': np.array([.02, .06, .30]),
        'yellow': np.array([.60, .50, .03]), 'orange': np.array([.60, .15, .02]),
        'white': np.array([.80, .80, .80])}
BOARD = np.array([.80, .80, .80])   # 표식 바탕 흰 판 가정

def detect(bgr, color, shape):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros((H, W), np.uint8)
    for lo, hi in R[color]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    mask = cv2.medianBlur(mask, 3)
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in cs:
        a = cv2.contourArea(c)
        if a < P['min_area_px']:
            continue
        per = cv2.arcLength(c, True)
        ap = cv2.approxPolyDP(c, P['approx_epsilon'] * per, True)
        x, y, w, h = cv2.boundingRect(ap)
        sh = classify_shape(len(ap), a, per, w, h, min_area=P['min_area_px'],
                            square_extent_min=P['square_extent_min'], square_aspect_lo=P['square_aspect_lo'],
                            square_aspect_hi=P['square_aspect_hi'],
                            circle_circularity_min=P['circle_circularity_min'],
                            square_circularity_max=P['square_circularity_max'])
        if sh != shape:
            continue
        v = ap.reshape(-1, 2); vx, vy = v[:, 0].mean(), v[:, 1].mean()
        if not (H * P['y_zone_lo'] <= vy <= H * P['y_zone_hi']):
            continue
        if best is None or a > best[0]:
            best = (a, vx)
    return best

def shape_mask(shape, cx, cy, s):
    m = np.zeros((H, W), np.uint8)
    if shape == 'Square':
        cv2.rectangle(m, (int(cx - s/2), int(cy - s/2)), (int(cx + s/2), int(cy + s/2)), 255, -1)
    elif shape == 'Circle':
        cv2.circle(m, (int(cx), int(cy)), int(s/2), 255, -1)
    else:
        pts = np.array([[cx, cy - s/2], [cx - s/2, cy + s/2], [cx + s/2, cy + s/2]], np.int32)
        cv2.fillPoly(m, [pts], 255)
    return cv2.GaussianBlur(m, (3, 3), 0).astype(float) / 255   # 렌즈 번짐 1px

def render(color, shape, size, k, illum=(1, 1, 1), flare=0.0, spec=0.0, hot=None, sky=None, rng=None):
    """k = 흰 판이 화면에서 갖는 선형 밝기(1.0=막 포화). illum = 표식면 조명 색. flare = 렌즈 베일(선형, 전 화면).
    spec = 표식면 정반사(선형 백색 가산). hot = (반사 점 반경 비율) 부분 정반사."""
    cx, cy = W / 2 + rng.uniform(-40, 40), H * 0.35 + rng.uniform(-8, 8)
    img = np.zeros((H, W, 3))
    img[:int(H*0.45)] = sky if sky is not None else np.array([.35, .45, .75]) * k * 1.2   # 하늘
    img[int(H*0.45):] = np.array([.05, .10, .12]) * k * 1.5                               # 물
    il = np.array(illum)
    bx = int(size * 1.4)
    img[int(cy-bx/2):int(cy+bx/2), int(cx-bx/2):int(cx+bx/2)] = BOARD * il * k + spec
    m = shape_mask(shape, cx, cy, size)[..., None]
    tgt = REFL[color] * il * k + spec
    img = img * (1 - m) + tgt * m
    if hot is not None:  # 표식 한쪽 가장자리에 걸친 반사 점
        hm = np.zeros((H, W), np.uint8)
        cv2.circle(hm, (int(cx + size*0.35), int(cy - size*0.2)), int(size*hot), 255, -1)
        hm = cv2.GaussianBlur(hm, (9, 9), 0).astype(float)[..., None] / 255
        img = img + hm * 1.5
    img = img + flare
    s = lin2s(img) * 255 + rng.normal(0, 2.0, img.shape)
    rgb = np.clip(s, 0, 255).astype(np.uint8)
    return rgb[..., ::-1].copy(), (cx, cy)

def target_hsv(color, k, illum=(1, 1, 1), flare=0.0, spec=0.0):
    lin = REFL[color] * np.array(illum) * k + spec + flare
    s = np.clip(lin2s(lin) * 255, 0, 255).astype(np.uint8)
    return cv2.cvtColor(s[::-1].reshape(1, 1, 3), cv2.COLOR_BGR2HSV)[0, 0]

def rate(color, shape, size, n=20, **kw):
    rng = np.random.default_rng(0)
    hit = 0
    for _ in range(n):
        bgr, (cx, _) = render(color, shape, size, rng=rng, **kw)
        b = detect(bgr, color, shape)
        if b is not None and abs(b[1] - cx) < size:
            hit += 1
    return hit / n

COLORS = ['red', 'green', 'blue', 'yellow', 'orange', 'white']
SHAPES = ['Square', 'Triangle', 'Circle']
SIZE = 50   # px. 표식 화면 크기 가정(접근 중간 거리)

def row(label, **kw):
    out = []
    for c in COLORS:
        r = min(rate(c, sh, SIZE, **kw) for sh in SHAPES)   # 세 도형 중 최저
        h = target_hsv(c, kw.get('k', 1), kw.get('illum', (1, 1, 1)), kw.get('flare', 0), kw.get('spec', 0))
        out.append(f"{int(r*100):3d}% ({h[0]:3d},{h[1]:3d},{h[2]:3d})")
    print(f"{label:34s} | " + " | ".join(out))


def main():
    SIZE = 50   # px. 표식 화면 크기 가정(접근 중간 거리)

    print("열: 색별 검출률(세 도형 최저, 20프레임) + 표식 중심 HSV")
    print(" " * 34 + " | " + " | ".join(f"{c:^20s}" for c in COLORS))
    print("— 기준: 실내 벤치 노출 재현(빨강 V≈92)")
    row("기준 k=0.45(실내 벤치 수준)", k=0.45)
    print("— 순광: 표식면에 햇빛 직사, 자동노출이 어두운 물·배 쪽에 맞춰 밝게")
    for k in (0.7, 1.0, 1.3, 1.8, 2.5):
        row(f"순광 노출 k={k}", k=k)
    print("— 순광 + 표식면 광택 정반사(전면 고르게)")
    for s in (0.05, 0.15, 0.3):
        row(f"순광 k=1.0 + 정반사 {s}", k=1.0, spec=s)
    print("— 순광 + 표식 일부에 반사 점(가장자리 걸침)")
    for hot in (0.15, 0.25, 0.35):
        row(f"순광 k=1.0 + 반사점 반경 {hot}", k=1.0, hot=hot)
    print("— 역광: 표식면 그늘(하늘빛 조명, 푸른 기), 자동노출이 밝은 하늘에 맞춤")
    blue_shade = (0.85, 1.0, 1.25)
    for k in (0.3, 0.2, 0.12, 0.08, 0.05):
        row(f"역광 k={k} 푸른그늘", k=k, illum=blue_shade, sky=np.array([1.5, 1.5, 1.6]))
    print("— 역광 + 렌즈 베일 플레어(전 화면 백색 가산)")
    for f in (0.02, 0.05, 0.1):
        row(f"역광 k=0.12 + 플레어 {f}", k=0.12, illum=blue_shade, flare=f, sky=np.array([1.5, 1.5, 1.6]))
    for f in (0.02, 0.05):
        row(f"역광 k=0.2 + 플레어 {f}", k=0.2, illum=blue_shade, flare=f, sky=np.array([1.5, 1.5, 1.6]))
    print("— 저녁 햇빛(주황 조명, 자동 WB 덜 보정)")
    row("석양 순광 k=1.0 조명(1.0,.8,.55)", k=1.0, illum=(1.0, 0.8, 0.55))
    row("석양 역광 k=0.15 조명(1.0,.8,.55)+플레어.03", k=0.15, illum=(1.0, 0.8, 0.55), flare=0.03, sky=np.array([1.5, 1.3, 1.0]))


if __name__ == '__main__':
    main()
