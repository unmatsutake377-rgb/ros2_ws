#!/usr/bin/env python3
"""dock_fsm_logic.median_min(전방 거리 공간 median) 단위 테스트 — ROS 없이 돈다.

실행: python3 src/ship_dock/test/test_dock_median_min.py
      tools/run_tests.sh ship_dock

계약서 docs/문제와작업/계약서_도킹물보라필터_20261109.md (11-09 승인):
  (a) 순수 함수 — 고립 스파이크 1·2 개 무시, 연속 3 점 벽 유지, 비스듬한 벽 raw 와 차 작음,
      kernel 0·1 = raw, 전부 inf → None, 고립만 → None, 짝수 kernel 창, lo>hi, 빈 배열.
  (b) 사본 동일성 — ship_direction/failsafe.py 원본을 **경로로** 로드해(import 의존 없음)
      무작위 입력(시드 고정, inf·0·음수·NaN, kernel 0~9, 경계 lo/hi) 수천 건에서 결과가 같은지.
      원본 파일이 없으면 skip 하지 않고 **실패**한다(사본이 혼자 남으면 어긋남을 못 잡는다).
"""

import array
import importlib.util
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from ship_dock.dock_fsm_logic import (  # noqa: E402
    median_min, median_kernel_window, MEDIAN_MIN_VALID, MEDIAN_KERNEL_MAX)

FAILSAFE_PATH = os.path.normpath(
    os.path.join(HERE, '..', '..', 'ship_direction', 'ship_direction', 'failsafe.py'))

INF = float('inf')
NAN = float('nan')


def raw_min(ranges, lo, hi):
    """11-09 이전 front_min_dist 의 raw 루프 그대로(대조용)."""
    lo = max(0, lo)
    hi = min(len(ranges) - 1, hi)
    if lo > hi:
        return None
    best = float('inf')
    for i in range(lo, hi + 1):
        r = ranges[i]
        if math.isfinite(r) and r > 0.0 and r < best:
            best = r
    return best if math.isfinite(best) else None


def wall(n=40, d=5.0):
    return [d] * n


# ─────────────── (a) 순수 함수 ───────────────
def test_constants():
    assert MEDIAN_MIN_VALID == 3          # 결정 2: 고정 상수
    assert MEDIAN_KERNEL_MAX == 9


def test_single_spike_ignored():
    r = wall()
    r[20] = 0.5                           # 물보라 한 점
    assert raw_min(r, 5, 35) == 0.5       # raw 는 속는다
    d, i = median_min(r, 5, 35, kernel=5, min_valid=3)
    assert d == 5.0, d


def test_two_spikes_ignored():
    for idx in ((20, 21), (10, 25)):      # 붙은 2 점 / 떨어진 2 점
        r = wall()
        for k in idx:
            r[k] = 0.4
        d, _ = median_min(r, 5, 35, kernel=5, min_valid=3)
        assert d == 5.0, (idx, d)


def test_three_point_object_kept():
    r = wall()
    r[19] = r[20] = r[21] = 0.8           # 연속 3 점 = 실제 물체
    d, i = median_min(r, 5, 35, kernel=5, min_valid=3)
    assert d == 0.8 and i in (19, 20, 21), (d, i)


def test_oblique_wall_close_to_raw():
    """비스듬한 벽(선형으로 멀어짐): median 은 raw 보다 크거나 같고, 차이는 half 칸의 기울기 이내."""
    slope = 0.01
    r = [2.0 + slope * k for k in range(60)]
    for lo, hi in ((10, 50), (0, 30)):    # 배열 안쪽 / 배열 끝(창이 잘림)
        raw = raw_min(r, lo, hi)
        d, _ = median_min(r, lo, hi, kernel=5, min_valid=3)
        assert d >= raw and (d - raw) <= 2 * slope + 1e-9, (lo, hi, raw, d)


def test_median_never_below_raw_when_window_inside_sector():
    """창이 섹터 안에 머무르면(섹터 = 배열 전체) median 은 raw 최소보다 작아질 수 없다."""
    rng = random.Random(7)
    for _ in range(1000):
        n = rng.randint(1, 30)
        r = [rng.choice([INF, NAN, 0.0, rng.uniform(0.1, 8.0)]) for _ in range(n)]
        raw = raw_min(r, 0, n - 1)
        d, _ = median_min(r, 0, n - 1, kernel=5, min_valid=3)
        if d is not None:
            assert raw is not None and d >= raw, (r, raw, d)


def test_sector_edge_window_reaches_outside():
    """원본 failsafe 와 같은 의미: 창은 섹터 밖 ±(kernel//2) 점까지 본다(배열 경계에서만 잘림).
    섹터 끝 안쪽이 무효점이면 섹터 바로 밖 값이 median 이 될 수 있다 → '항상 median ≥ 섹터 raw' 는
    섹터 끝 kernel//2 점(실제 라이다 약 0.5°)에서는 성립하지 않는다. 이 동작을 고정해 둔다."""
    r = [1.0, 1.0, 5.0, INF, INF, 5.0, 5.0, 5.0]
    assert raw_min(r, 2, 7) == 5.0
    assert median_min(r, 2, 7, kernel=5, min_valid=3) == (1.0, 2)


def test_kernel_0_1_equal_raw():
    rng = random.Random(11)
    for _ in range(500):
        n = rng.randint(0, 30)
        r = [rng.choice([INF, -INF, NAN, 0.0, -1.0, rng.uniform(0.05, 10.0)]) for _ in range(n)]
        lo, hi = rng.randint(-3, n + 1), rng.randint(-3, n + 3)
        for k in (0, 1):
            d, _ = median_min(r, lo, hi, kernel=k, min_valid=3)
            assert d == raw_min(r, lo, hi), (k, r, lo, hi, d)
    r = wall()
    r[20] = 0.5
    assert median_min(r, 5, 35, kernel=1)[0] == 0.5     # 롤백: 스파이크에 다시 속는다
    assert median_min(r, 5, 35, kernel=1, return_rejected=True)[2] == 0


def test_all_inf_none():
    assert median_min([INF] * 40, 5, 35, kernel=5) == (None, None)
    assert median_min([INF] * 40, 5, 35, kernel=5, return_rejected=True) == (None, None, 0)


def test_invalid_values_ignored():
    r = [NAN, 0.0, -2.0, -INF, INF] * 8
    assert median_min(r, 0, 39, kernel=5) == (None, None)
    assert median_min(r, 0, 39, kernel=1) == (None, None)


def test_isolated_only_none_with_rejected_count():
    r = [INF] * 40
    for k in (8, 20, 32):                 # 서로 떨어진 고립 반사 3 점
        r[k] = 3.0
    assert raw_min(r, 5, 35) == 3.0
    d, i, rej = median_min(r, 5, 35, kernel=5, min_valid=3, return_rejected=True)
    assert (d, i, rej) == (None, None, 3), (d, i, rej)


def test_even_kernel_window():
    assert median_kernel_window(0) == 1 and median_kernel_window(1) == 1
    assert median_kernel_window(2) == 3 and median_kernel_window(3) == 3
    assert median_kernel_window(4) == 5 and median_kernel_window(5) == 5
    assert median_kernel_window(8) == 9 and median_kernel_window(9) == 9
    for bad in (-1, 10, 100, 5.0, None, True, '5'):
        assert median_kernel_window(bad) is None, bad
    rng = random.Random(3)
    for _ in range(300):
        n = rng.randint(0, 25)
        r = [rng.choice([INF, NAN, 0.0, rng.uniform(0.1, 6.0)]) for _ in range(n)]
        lo, hi = rng.randint(-2, n), rng.randint(-2, n + 2)
        for k_even in (2, 4, 6, 8):       # 짝수 k 는 창 k+1 = 홀수 k+1 과 같은 결과
            a = median_min(r, lo, hi, kernel=k_even, return_rejected=True)
            b = median_min(r, lo, hi, kernel=k_even + 1, return_rejected=True)
            assert a == b, (k_even, r, lo, hi)


def test_lo_gt_hi_and_empty():
    assert median_min(wall(), 30, 10, kernel=5) == (None, None)
    assert median_min(wall(), 30, 10, kernel=5, return_rejected=True) == (None, None, 0)
    assert median_min([], 0, 5, kernel=5) == (None, None)
    assert median_min([], 0, 5, kernel=1) == (None, None)
    assert median_min(wall(10), 15, 20, kernel=5) == (None, None)     # 섹터가 배열 밖


def test_float32_array_input():
    """rclpy LaserScan.ranges 는 array.array('f') 다 — 리스트와 같은 결과."""
    r = wall()
    r[20] = 0.5
    r[30] = INF
    a = array.array('f', r)
    assert median_min(a, 5, 35, kernel=5) == median_min(r, 5, 35, kernel=5)


# ─────────────── (b) 사본 ↔ ship_direction/failsafe.py 원본 동일성 ───────────────
def _load_failsafe():
    assert os.path.isfile(FAILSAFE_PATH), (
        f"원본 {FAILSAFE_PATH} 가 없다 — 사본 동일성을 확인할 수 없다(skip 아님, 실패)")
    spec = importlib.util.spec_from_file_location('ssf_failsafe_original_for_dock', FAILSAFE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert hasattr(mod, 'median_min'), "원본 failsafe.py 에 median_min 이 없다"
    return mod


def _rand_value(rng):
    roll = rng.random()
    if roll < 0.10:
        return INF
    if roll < 0.14:
        return -INF
    if roll < 0.20:
        return NAN
    if roll < 0.25:
        return 0.0
    if roll < 0.30:
        return -rng.uniform(0.01, 5.0)
    if roll < 0.40:
        return rng.choice([1.0, 2.5, 3.0])          # 같은 값(동률 인덱스) 자주
    return rng.uniform(0.01, 12.0)


def _same(a, b):
    """튜플 비교. 거리는 NaN 이 나올 수 없지만(필터됨) 혹시 나오면 둘 다 NaN 이면 같게 본다."""
    if len(a) != len(b):
        return False
    for x, y in zip(a, b):
        if isinstance(x, float) and isinstance(y, float) and math.isnan(x) and math.isnan(y):
            continue
        if x != y:
            return False
    return True


def test_copy_equals_failsafe_original():
    orig = _load_failsafe().median_min
    rng = random.Random(20261109)
    cases = 0
    for _ in range(4000):
        n = rng.randint(0, 40)
        r = [_rand_value(rng) for _ in range(n)]
        if rng.random() < 0.2 and n:                 # 고립 스파이크·짧은 덩어리 섞기
            base = rng.uniform(2.0, 8.0)
            r = [base if rng.random() < 0.8 else INF for _ in range(n)]
            for _k in range(rng.randint(1, 3)):
                s = rng.randrange(n)
                for j in range(s, min(n, s + rng.randint(1, 3))):
                    r[j] = rng.uniform(0.2, 1.5)
        lo = rng.randint(-5, n + 5)
        hi = rng.choice([rng.randint(-5, n + 5), n - 1, n, lo, lo - 1])
        kernel = rng.randint(0, MEDIAN_KERNEL_MAX)
        for mv in (MEDIAN_MIN_VALID, rng.randint(1, 5)):
            for rr in (False, True):
                a = median_min(r, lo, hi, kernel=kernel, min_valid=mv, return_rejected=rr)
                b = orig(r, lo, hi, kernel=kernel, min_valid=mv, return_rejected=rr)
                assert _same(a, b), (r, lo, hi, kernel, mv, rr, a, b)
                cases += 1
    assert cases >= 4000 * 4


def test_copy_equals_failsafe_on_boundaries():
    """경계 고정: lo/hi 가 배열 끝·밖, kernel 0~9 전부, 기본 인자(kernel 5, min_valid 3)."""
    orig = _load_failsafe().median_min
    r = [5.0, INF, 0.5, 5.0, 5.0, NAN, 0.0, 5.0, -1.0, 0.7, 0.7, 0.7, 5.0]
    n = len(r)
    for kernel in range(0, MEDIAN_KERNEL_MAX + 1):
        for lo in (-3, 0, 1, n - 1, n, n + 2):
            for hi in (-1, 0, n - 2, n - 1, n, n + 4):
                assert median_min(r, lo, hi, kernel) == orig(r, lo, hi, kernel), (kernel, lo, hi)
    assert median_min(r, 0, n - 1) == orig(r, 0, n - 1)
    assert median_min(r, 0, n - 1, return_rejected=True) == orig(r, 0, n - 1, return_rejected=True)


if __name__ == '__main__':
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith('test_') and callable(v)]
    fails = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  ✅ {name}")
        except AssertionError as e:
            fails += 1
            print(f"  ❌ {name}\n     {e!r}"[:400])
    print(f"{len(tests) - fails}/{len(tests)} 통과")
    sys.exit(1 if fails else 0)
