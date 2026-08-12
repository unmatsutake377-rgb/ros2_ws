#!/usr/bin/env python3
"""hsv_ranges 순수 로직 테스트 — ROS·OpenCV·카메라 없이 돈다."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from color_shape_detector.hsv_ranges import (  # noqa: E402
    DEFAULT_RANGES, SUPERSEDED, VALID_COLORS, HsvRangeError,
    default_flat, flatten, format_yaml, load, parse_flat, widen_to_include,
)


class TestParse(unittest.TestCase):

    def test_한_범위(self):
        r = parse_flat([60, 120, 120, 85, 255, 255], "green")
        self.assertEqual(r, [((60, 120, 120), (85, 255, 255))])

    def test_두_범위_빨강(self):
        r = parse_flat([0, 140, 80, 5, 255, 255, 165, 140, 80, 180, 255, 255], "red")
        self.assertEqual(len(r), 2)
        self.assertEqual(r[1][0], (165, 140, 80))

    def test_왕복(self):
        for c in VALID_COLORS:
            self.assertEqual(parse_flat(default_flat(c), c), DEFAULT_RANGES[c])

    # ── 거부해야 하는 것들 (조용히 넘기지 않는다) ──

    def test_거부_None(self):
        with self.assertRaises(HsvRangeError):
            parse_flat(None)

    def test_거부_빈값(self):
        with self.assertRaises(HsvRangeError):
            parse_flat([])

    def test_거부_길이가_6배수아님(self):
        with self.assertRaises(HsvRangeError):
            parse_flat([0, 140, 80, 5, 255])

    def test_거부_정수아님(self):
        with self.assertRaises(HsvRangeError):
            parse_flat([0, "빨강", 80, 5, 255, 255])

    def test_거부_H_범위밖(self):
        with self.assertRaises(HsvRangeError):
            parse_flat([200, 140, 80, 210, 255, 255])

    def test_거부_S_범위밖(self):
        with self.assertRaises(HsvRangeError):
            parse_flat([0, 300, 80, 5, 255, 255])

    def test_거부_S_하한이_상한보다_큼(self):
        with self.assertRaises(HsvRangeError):
            parse_flat([0, 200, 80, 5, 100, 255])

    def test_거부_V_하한이_상한보다_큼(self):
        with self.assertRaises(HsvRangeError):
            parse_flat([0, 100, 200, 5, 255, 100])

    def test_H는_대소를_강제하지_않는다(self):
        """🚨 빨강은 색상환 0을 넘어가서 두 범위로 쪼갠다 — H 대소 강제는 틀린 검사다."""
        parse_flat([170, 140, 80, 10, 255, 255])   # 예외 없이 통과해야 한다


class TestDefaults(unittest.TestCase):

    def test_모든_색이_파싱된다(self):
        for c in VALID_COLORS:
            self.assertTrue(parse_flat(default_flat(c), c))

    def test_초록은_연두가_아니다(self):
        """🚨 dock 에 있던 H 28~40(연두)로 되돌아가는 것을 막는다."""
        lo, hi = DEFAULT_RANGES["green"][0]
        self.assertGreaterEqual(lo[0], 50, "H 하한이 초록대(50+)를 벗어났다")
        self.assertLessEqual(hi[0], 95)

    def test_초록은_선명한_초록을_거부하지_않는다(self):
        """🚨 dock 값은 S≤100 이라 선명한 초록을 놓쳤다."""
        _, hi = DEFAULT_RANGES["green"][0]
        self.assertEqual(hi[1], 255, "S 상한이 255 가 아니면 선명한 초록을 못 잡는다")

    def test_빨강은_어두운_빨강도_잡는다(self):
        """🚨 dock 값은 V≥200 이라 그늘의 빨강을 놓쳤다."""
        lo, _ = DEFAULT_RANGES["red"][0]
        self.assertLessEqual(lo[2], 100, f"V 하한 {lo[2]} 이 너무 높다 — 어두운 빨강을 놓친다")

    def test_버린_값도_파싱은_된다(self):
        """비교하려면 읽을 수는 있어야 한다."""
        for k, flat in SUPERSEDED.items():
            self.assertTrue(parse_flat(flat, k))


class TestLoad(unittest.TestCase):

    def test_파라미터가_없으면_기본값(self):
        got = load(lambda name, default: default, ["red", "green"])
        self.assertEqual(got["red"], DEFAULT_RANGES["red"])
        self.assertEqual(got["green"], DEFAULT_RANGES["green"])

    def test_파라미터가_있으면_그걸_쓴다(self):
        custom = [10, 20, 30, 40, 50, 60]
        got = load(lambda name, default: custom if name == "hsv.red" else default,
                   ["red", "green"])
        self.assertEqual(got["red"], [((10, 20, 30), (40, 50, 60))])
        self.assertEqual(got["green"], DEFAULT_RANGES["green"])

    def test_잘못된_값이면_기본값으로_되돌리고_알린다(self):
        msgs = []
        got = load(lambda name, default: [1, 2, 3] if name == "hsv.red" else default,
                   ["red"], on_error=msgs.append)
        self.assertEqual(got["red"], DEFAULT_RANGES["red"])
        self.assertEqual(len(msgs), 1)
        self.assertIn("HSV", msgs[0])

    def test_검출기가_쓰는_모양_그대로다(self):
        """기존 하드코딩 dict 와 같은 모양이어야 갈아끼울 수 있다: {색: [(lo,hi), …]}"""
        got = load(lambda n, d: d, ["red"])
        for lo, hi in got["red"]:
            self.assertEqual(len(lo), 3)
            self.assertEqual(len(hi), 3)


class TestFormatYaml(unittest.TestCase):

    def test_붙여넣기_가능한_형태(self):
        line = format_yaml("green", DEFAULT_RANGES["green"])
        self.assertIn("hsv.green:", line)
        self.assertIn("60", line)

    def test_찍은_것을_다시_읽을_수_있다(self):
        """도구가 찍은 줄을 yaml 에 넣으면 그대로 파싱돼야 한다 — 왕복이 깨지면 무용지물이다."""
        for c in VALID_COLORS:
            line = format_yaml(c, DEFAULT_RANGES[c])
            inner = line.split("[", 1)[1].rsplit("]", 1)[0]
            vals = [int(x) for x in inner.replace(" ", "").split(",")]
            self.assertEqual(parse_flat(vals, c), DEFAULT_RANGES[c])


class TestWiden(unittest.TestCase):

    def test_이미_들어오는_픽셀이면_거의_안_변한다(self):
        base = [((60, 120, 120), (85, 255, 255))]
        out = widen_to_include(base, (70, 200, 200), margin=(0, 0, 0))
        self.assertEqual(out, base)

    def test_밖의_픽셀을_품도록_넓어진다(self):
        base = [((60, 120, 120), (85, 255, 255))]
        out = widen_to_include(base, (50, 100, 100), margin=(5, 10, 10))
        lo, hi = out[0]
        self.assertLessEqual(lo[0], 50)
        self.assertLessEqual(lo[1], 100)
        self.assertLessEqual(lo[2], 100)

    def test_경계를_안_넘는다(self):
        base = [((0, 0, 0), (180, 255, 255))]
        out = widen_to_include(base, (0, 0, 0), margin=(50, 50, 50))
        lo, hi = out[0]
        self.assertGreaterEqual(lo[0], 0)
        self.assertGreaterEqual(lo[1], 0)
        self.assertLessEqual(hi[1], 255)
        self.assertLessEqual(hi[2], 255)

    def test_두_범위중_가까운_쪽을_넓힌다(self):
        """빨강처럼 범위가 둘이면 엉뚱한 쪽을 넓히면 안 된다.

        H=160 은 두 범위 **밖**이고 165~180 쪽에 더 가깝다(거리 5 vs 20).
        """
        base = parse_flat(default_flat("red"), "red")
        out = widen_to_include(base, (160, 200, 200), margin=(5, 10, 10))
        self.assertEqual(out[0], base[0], "0~5 쪽 범위는 건드리지 말아야 한다")
        self.assertNotEqual(out[1], base[1], "165~180 쪽이 넓어져야 한다")
        self.assertLessEqual(out[1][0][0], 160, "H 하한이 160 을 품어야 한다")

    def test_이미_품고_있으면_그대로_둔다(self):
        """H=170 은 이미 165~180 안이다 — 괜히 넓히면 인접색을 끌어온다."""
        base = parse_flat(default_flat("red"), "red")
        out = widen_to_include(base, (170, 200, 200), margin=(5, 10, 10))
        self.assertEqual(out, base)

    def test_빈_범위는_거부(self):
        with self.assertRaises(HsvRangeError):
            widen_to_include([], (10, 10, 10))



class TestSyncWithDockLogic(unittest.TestCase):
    """🚨 dock_logic 의 유효색 목록과 HSV 표가 어긋나면 조용히 실패한다.

    target_color 로는 통과하는데 HSV 범위가 없으면 `ranges is None` 으로 빠져
    **에러 없이 아무것도 못 찾는다.** 도킹이 1년간 침묵한 것과 같은 유형이다.
    """

    def test_유효색_목록이_같다(self):
        from color_shape_detector.dock_logic import VALID_COLORS as DOCK_COLORS
        self.assertEqual(set(DOCK_COLORS), set(VALID_COLORS),
                         "dock_logic.VALID_COLORS 와 hsv_ranges.VALID_COLORS 가 어긋났다")

    def test_모든_유효색에_범위가_있다(self):
        from color_shape_detector.dock_logic import VALID_COLORS as DOCK_COLORS
        for c in DOCK_COLORS:
            self.assertIn(c, DEFAULT_RANGES, f"'{c}' 는 목표색인데 HSV 범위가 없다")

if __name__ == '__main__':
    unittest.main()
