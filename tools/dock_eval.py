#!/usr/bin/env python3
"""
dock_eval — 도크 표식 인식 평가 하네스 (도크미션 작업계획 D4).

무엇:
  rosbag(카메라 프레임 기록)을 프레임별로 재생하며 dock_logic 의 형상 분류를 돌리고,
  정답(라벨) 대비 **형상×색 혼동행렬(confusion matrix)** 을 CSV 로 낸다.
  D2·D3 의 파라미터(confirm_frames, circularity 임계 등)를 **감이 아니라 이 수치로** 튜닝한다.

🚨 지금 상태:
  · 작년 rosbag 자산이 아직 없어서(확인함) rosbag 재생 경로는 **골격만** 있다.
  · rosbag 이 확보되면 `--bag` 으로 돌린다.
  · 지금 당장은 `--selftest` 로 dock_logic 분류 로직 자체를 합성 특징으로 검증할 수 있다
    (ROS/cv2 불필요 — Mac 에서도 실행됨).

사용:
  python3 tools/dock_eval.py --selftest
  python3 tools/dock_eval.py --bag <rosbag경로> --labels <labels.csv> --out confusion.csv   # rosbag 확보 후

labels.csv 형식 (rosbag 프레임 인덱스별 정답):
  frame_idx,color,shape
  0,red,Square
  1,red,Square
  ...
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'src', 'color_shape_detector'))

from color_shape_detector.dock_logic import (  # noqa: E402
    VALID_SHAPES, classify_shape, DetectionConfirmer,
)


# ────────────────────────────── 혼동행렬 ──────────────────────────────
class Confusion:
    """정답 shape → 예측 shape 카운트. 미검출은 'None' 열로."""

    def __init__(self):
        self.labels = list(VALID_SHAPES) + ["None"]
        self.mat = {a: {b: 0 for b in self.labels} for a in VALID_SHAPES}

    def add(self, truth, pred):
        if truth not in self.mat:
            return
        self.mat[truth][pred if pred in self.labels else "None"] += 1

    def to_csv(self):
        lines = ["truth\\pred," + ",".join(self.labels)]
        for a in VALID_SHAPES:
            row = [str(self.mat[a][b]) for b in self.labels]
            lines.append(a + "," + ",".join(row))
        return "\n".join(lines) + "\n"

    def accuracy(self):
        correct = sum(self.mat[a][a] for a in VALID_SHAPES)
        total = sum(self.mat[a][b] for a in VALID_SHAPES for b in self.labels)
        return (correct / total) if total else 0.0


# ────────────────────────────── 셀프테스트 ──────────────────────────────
def _feat(shape, size=60):
    """합성 기하 특징 (cv2 없이). classify_shape 검증용."""
    import math
    if shape == "Circle":
        r = size / 2
        return dict(v_count=12, area=math.pi * r * r, perimeter=2 * math.pi * r,
                    w=size, h=size)
    if shape == "Square":
        return dict(v_count=4, area=size * size, perimeter=4 * size, w=size, h=size)
    # Triangle
    a = math.sqrt(3) / 4 * size * size
    return dict(v_count=3, area=a, perimeter=3 * size, w=size, h=math.sqrt(3) / 2 * size)


def run_selftest():
    print("=== dock_eval 셀프테스트 (합성 특징) ===")
    conf = Confusion()
    for shape in VALID_SHAPES:
        for size in (20, 40, 80, 160):
            pred = classify_shape(**_feat(shape, size))
            conf.add(shape, pred if pred else "None")
    print(conf.to_csv())
    acc = conf.accuracy()
    print(f"정확도: {acc*100:.1f}%")

    # confirmer 데모: 오탐 1프레임이 확정을 못 뚫는지
    c = DetectionConfirmer(confirm_frames=3)
    seq = [("red", "Square")] * 3 + [("green", "Triangle")] + [("red", "Square")] * 2
    outs = [c.update(k) for k in seq]
    print(f"confirmer 시퀀스 결과: {outs}")
    print("  (오탐 green,Triangle 프레임에서 None 이면 정상)")

    return 0 if acc == 1.0 else 1


# ────────────────────────────── rosbag 경로 (골격) ──────────────────────────────
def run_bag(bag_path, labels_path, out_path):
    print(f"[dock_eval] rosbag 평가: {bag_path}")
    # ⚠️ 골격이다. rosbag 확보 후 아래를 채운다:
    #   1) rosbag2_py 로 bag 열기, 이미지 토픽 이터레이트
    #   2) 각 프레임을 cv_bridge 로 디코드
    #   3) basic_image_subscriberdock.process_image 와 '같은 전처리'로 컨투어 특징 추출
    #      (HSV 마스크 → findContours → area/perimeter/boundingRect)
    #   4) classify_shape 로 예측, labels.csv 의 정답과 대조 → Confusion.add
    #   5) Confusion.to_csv() 를 out_path 에 기록
    # 지금은 rosbag 자산이 없어 미구현. 확보되면 이 함수만 채우면 된다.
    raise NotImplementedError(
        "rosbag 재생은 아직 미구현(작년 bag 자산 없음). "
        "rosbag 확보 후 이 함수를 채운다. 지금은 --selftest 로 분류 로직을 검증하라.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true", help="합성 특징으로 분류 로직 검증")
    ap.add_argument("--bag", help="rosbag 경로 (확보 후)")
    ap.add_argument("--labels", help="정답 CSV")
    ap.add_argument("--out", default="dock_confusion.csv", help="혼동행렬 출력 CSV")
    args = ap.parse_args()

    if args.selftest or not args.bag:
        return run_selftest()
    return run_bag(args.bag, args.labels, args.out)


if __name__ == "__main__":
    sys.exit(main())
