#!/usr/bin/env python3
"""turn 검출기 모드별 목표 색(C6/U5) — ROS·OpenCV 없이 돈다.

순수 로직(turn_color_logic) + vision.yaml 값 + 노드 소스 정적 검사(ast, import 하지 않음).
"""

import ast
import os
import sys
import unittest

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from color_shape_detector.hsv_ranges import VALID_COLORS  # noqa: E402
from color_shape_detector.turn_color_logic import (  # noqa: E402
    DEFAULT_TURN_COLOR_MODE2, DEFAULT_TURN_COLOR_MODE3, INVALID_LOG_PERIOD_S, NO_ORBIT_DIRECTION,
    TURN_COLORS, build_wp_color_map, detection_disabled, log_due, orbit_direction_warning,
    resolve_turn_color, target_colors_for,
)

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
YAML_PATH = os.path.join(PKG, 'config', 'vision.yaml')
NODE_PATH = os.path.join(PKG, 'color_shape_detector', 'basic_image_subscriberturn.py')
# ship_turn 의 색 → 방향 함수(소스 트리에서만 있다 — 설치 트리에서 돌면 동기화 테스트는 skip).
ORBIT_LOGIC_PATH = os.path.join(PKG, '..', 'ship_turn', 'ship_turn', 'orbit_logic.py')

# 옛 하드코딩 맵 — 기본값일 때 이것과 똑같아야 한다(동작 불변).
LEGACY_MAP = {2: ["white"], 3: ["red"], 4: ["red"], 5: ["red"]}


def _turn_params():
    with open(YAML_PATH, encoding='utf-8') as f:
        return yaml.safe_load(f)['image_subscriber_turn']['ros__parameters']


class TestDefaults(unittest.TestCase):

    def test_코드_기본값은_yaml과_같다(self):
        p = _turn_params()
        self.assertEqual(p['turn_color_mode2'], DEFAULT_TURN_COLOR_MODE2)
        self.assertEqual(p['turn_color_mode3'], DEFAULT_TURN_COLOR_MODE3)

    def test_기본값은_옛_값(self):
        self.assertEqual(DEFAULT_TURN_COLOR_MODE2, "white")
        self.assertEqual(DEFAULT_TURN_COLOR_MODE3, "red")

    def test_기본_맵은_옛_하드코딩과_같다(self):
        m = build_wp_color_map(DEFAULT_TURN_COLOR_MODE2, DEFAULT_TURN_COLOR_MODE3)
        self.assertEqual(m, LEGACY_MAP)

    def test_yaml_값으로_만든_맵도_옛_맵과_같다(self):
        p = _turn_params()
        c2, e2 = resolve_turn_color(p['turn_color_mode2'])
        c3, e3 = resolve_turn_color(p['turn_color_mode3'])
        self.assertIsNone(e2)
        self.assertIsNone(e3)
        self.assertEqual(build_wp_color_map(c2, c3), LEGACY_MAP)

    def test_허용색은_HSV를_아는_색(self):
        for c in TURN_COLORS:
            self.assertIn(c, VALID_COLORS)


class TestResolve(unittest.TestCase):

    def test_허용색_통과(self):
        for c in TURN_COLORS:
            self.assertEqual(resolve_turn_color(c), (c, None))

    def test_대소문자_공백_정리(self):
        self.assertEqual(resolve_turn_color(" Red "), ("red", None))
        self.assertEqual(resolve_turn_color("WHITE"), ("white", None))
        self.assertEqual(resolve_turn_color("\tGreen\n"), ("green", None))

    def test_모르는_색은_None과_오류_기본값_대체_없음(self):
        # orange 는 hsv_ranges 엔 있지만 이 검출기가 HSV 를 읽지 않는다 → 받아 주면 조용히 못 찾는다.
        # 기본 red 로 바꿔 주지도 않는다 — 오타로 게이트 빨강에 HOLD 하는 것을 막는다.
        for bad in ("orange", "빨강", "", "  ", "re d", "rde"):
            c, err = resolve_turn_color(bad)
            self.assertIsNone(c, bad)
            self.assertIsNotNone(err, bad)
            self.assertIn("검출하지 않는다", err)

    def test_문자열이_아니면_None과_오류(self):
        # 실제 노드에선 rclpy 가 declare_parameter 타입 오류로 먼저 세운다. 로직도 막아 둔다.
        for bad in (None, 3, ["red"]):
            c, err = resolve_turn_color(bad)
            self.assertIsNone(c)
            self.assertIsNotNone(err)

    def test_mode3만_바꾸면_4_5도_따른다(self):
        m = build_wp_color_map("white", "green")
        self.assertEqual(m, {2: ["white"], 3: ["green"], 4: ["green"], 5: ["green"]})


class Test2026Colors(unittest.TestCase):
    """2026-11 경기규정(탐색 노랑 = 좌현선회) + 11-11 전달(위치유지 검정) — 계약서_탐색선회_부표색_20261111 C1.

    2026 OT(11-12): 검정 = 탐색 좌현선회(반시계)도 된다 — 계약서_탐색검정좌현_20261112 K2.
    """

    def test_노랑_검정을_받는다(self):
        self.assertEqual(resolve_turn_color(" Yellow "), ("yellow", None))
        self.assertEqual(resolve_turn_color("BLACK"), ("black", None))

    def test_위치유지_후보와_탐색_색이_모두_허용(self):
        for c in ("green", "red", "black", "yellow"):
            self.assertIn(c, TURN_COLORS)

    def test_검정_위치유지_맵(self):
        m = build_wp_color_map("black", "yellow")
        self.assertEqual(m, {2: ["black"], 3: ["yellow"], 4: ["yellow"], 5: ["yellow"]})
        self.assertFalse(detection_disabled(2, m))

    def test_방향_없는_색은_허용_색의_부분집합(self):
        for c in NO_ORBIT_DIRECTION:
            self.assertIn(c, TURN_COLORS)

    def test_mode3_검정이면_WARN_없음(self):
        # 2026 OT(11-12): 검정 = 좌현선회. 수정 전엔 NO_ORBIT_DIRECTION 에 있어 거짓 WARN 을 냈다.
        self.assertIsNone(orbit_direction_warning("black"))

    def test_WARN_장치는_남아_있다(self):
        # NO_ORBIT_DIRECTION 은 지금 빈 튜플이지만 장치(색이 들어오면 WARN)는 유지한다.
        mod = sys.modules[orbit_direction_warning.__module__]
        saved = mod.NO_ORBIT_DIRECTION
        mod.NO_ORBIT_DIRECTION = ("black",)
        try:
            w = orbit_direction_warning("black")
        finally:
            mod.NO_ORBIT_DIRECTION = saved
        self.assertIsNotNone(w)
        self.assertIn("SEARCH", w)

    def test_mode3_방향_있는_색이나_무효면_WARN_없음(self):
        for c in ("red", "green", "yellow", "white", "black", None):
            self.assertIsNone(orbit_direction_warning(c), c)

    def test_방향_없는_색이_ship_turn_과_같다(self):
        # 상수 공유 모듈은 만들지 않는다(CLAUDE.md 1-8) — 대신 ship_turn 소스를 ast 로 읽어 함수만 실행해 맞춘다.
        if not os.path.isfile(ORBIT_LOGIC_PATH):
            self.skipTest('ship_turn 소스 없음(설치 트리)')
        with open(ORBIT_LOGIC_PATH, encoding='utf-8') as f:
            tree = ast.parse(f.read())
        fn = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'orbit_direction_cw']
        self.assertEqual(len(fn), 1, 'orbit_logic.orbit_direction_cw 를 못 찾았다')
        ns = {}
        exec(compile(ast.Module(body=fn, type_ignores=[]), ORBIT_LOGIC_PATH, 'exec'), ns)
        for c in TURN_COLORS:
            self.assertEqual(ns['orbit_direction_cw'](c) is None, c in NO_ORBIT_DIRECTION, c)

    def test_노드가_부팅때_WARN_을_부른다(self):
        with open(NODE_PATH, encoding='utf-8') as f:
            src = f.read()
        self.assertIn('orbit_direction_warning(self.turn_color_mode3)', src)


class TestInvalidDisables(unittest.TestCase):
    """무효 색 → 그 모드는 어떤 색도 통과하지 않고 발행도 하지 않는다."""

    ALL = ("red", "green", "white")

    def _map(self, raw2, raw3):
        return build_wp_color_map(resolve_turn_color(raw2)[0], resolve_turn_color(raw3)[0])

    def test_mode3_무효면_3_4_5_빈_목록_키는_남는다(self):
        m = self._map("white", "orang")
        self.assertEqual(m, {2: ["white"], 3: [], 4: [], 5: []})

    def test_mode2_무효면_2만_빈_목록(self):
        m = self._map("whte", "red")
        self.assertEqual(m, {2: [], 3: ["red"], 4: ["red"], 5: ["red"]})

    def test_빈_목록이면_아무_색도_통과하지_않는다(self):
        m = self._map("white", "orang")
        for mode in (3, 4, 5):
            tc = target_colors_for(mode, m, self.ALL)
            self.assertEqual([c for c in self.ALL if c in tc], [], mode)   # process_image 의 `in`
            self.assertTrue(detection_disabled(mode, m), mode)
        self.assertEqual(target_colors_for(2, m, self.ALL), ["white"])
        self.assertFalse(detection_disabled(2, m))

    def test_맵에_없는_모드는_모든_색_그리고_검출함(self):
        m = self._map("white", "orang")
        self.assertEqual(target_colors_for(7, m, self.ALL), list(self.ALL))
        self.assertFalse(detection_disabled(7, m))
        self.assertFalse(detection_disabled(-1, m))

    def test_기본_맵은_검출을_끄지_않는다(self):
        m = build_wp_color_map(DEFAULT_TURN_COLOR_MODE2, DEFAULT_TURN_COLOR_MODE3)
        for mode in (2, 3, 4, 5):
            self.assertFalse(detection_disabled(mode, m))
            self.assertEqual(target_colors_for(mode, m, self.ALL), LEGACY_MAP[mode])


class TestLogDue(unittest.TestCase):
    """무효 색 모드의 주기 ERROR — 5 s 마다 1줄(dock 의 TARGET_INVALID_LOG_PERIOD_S 와 같은 판정)."""

    def test_주기는_5초(self):
        self.assertEqual(INVALID_LOG_PERIOD_S, 5.0)

    def test_처음엔_바로_찍는다(self):
        self.assertTrue(log_due(None, 0.0))
        self.assertTrue(log_due(None, 12345.6))

    def test_주기_안에서는_안_찍는다(self):
        self.assertFalse(log_due(100.0, 100.0))
        self.assertFalse(log_due(100.0, 104.9))
        self.assertFalse(log_due(100.0, 105.0))   # 경계는 거짓(`>`)

    def test_주기가_지나면_찍는다(self):
        self.assertTrue(log_due(100.0, 105.01))

    def test_프레임_흐름에서_5초에_1번(self):
        # 30 fps 로 12 s → 0, 5.0x, 10.0x 에 3번
        last, n = None, 0
        for i in range(360):
            now = 1000.0 + i / 30.0
            if log_due(last, now):
                last, n = now, n + 1
        self.assertEqual(n, 3)


class TestNodeSource(unittest.TestCase):
    """노드가 위 로직을 실제로 쓰는지 — 하드코딩 맵으로 되돌아가지 않게."""

    @classmethod
    def setUpClass(cls):
        with open(NODE_PATH, encoding='utf-8') as f:
            cls.src = f.read()
        cls.tree = ast.parse(cls.src, filename=NODE_PATH)

    def _init(self):
        for n in ast.walk(self.tree):
            if isinstance(n, ast.FunctionDef) and n.name == '__init__':
                return n
        self.fail("__init__ 없음")

    def test_하드코딩_맵_없음(self):
        for n in ast.walk(self.tree):
            if isinstance(n, ast.Assign) and isinstance(n.value, ast.Dict):
                for t in n.targets:
                    self.assertFalse(isinstance(t, ast.Attribute) and t.attr == 'wp_color_map',
                                     "wp_color_map 이 dict 리터럴로 박혀 있다")

    def test_파라미터를_기본값_상수로_선언(self):
        calls = [n for n in ast.walk(self._init())
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                 and n.func.attr == '_turn_color_param']
        got = {(c.args[0].value, c.args[1].id) for c in calls}
        self.assertEqual(got, {('turn_color_mode2', 'DEFAULT_TURN_COLOR_MODE2'),
                               ('turn_color_mode3', 'DEFAULT_TURN_COLOR_MODE3')})

    def test_색_선언이_구독보다_먼저(self):
        init = self._init()
        decl = [n.lineno for n in ast.walk(init)
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == '_turn_color_param']
        subs = [n.lineno for n in ast.walk(init)
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'create_subscription']
        self.assertTrue(decl and subs)
        self.assertLess(max(decl), min(subs))

    def _func(self, name):
        for n in ast.walk(self.tree):
            if isinstance(n, ast.FunctionDef) and n.name == name:
                return n
        self.fail(f"{name} 없음")

    def _call_lines(self, fn, attr=None, name=None):
        out = []
        for n in ast.walk(fn):
            if not isinstance(n, ast.Call):
                continue
            if attr and isinstance(n.func, ast.Attribute) and n.func.attr == attr:
                out.append(n.lineno)
            if name and isinstance(n.func, ast.Name) and n.func.id == name:
                out.append(n.lineno)
        return out

    def test_resolve_에_기본값을_넘기지_않는다(self):
        # 기본값 대체가 되살아나지 않게 — resolve_turn_color 는 인자 1개로만 부른다.
        calls = [n for n in ast.walk(self.tree)
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                 and n.func.id == 'resolve_turn_color']
        self.assertTrue(calls)
        for c in calls:
            self.assertEqual(len(c.args), 1)
            self.assertEqual(c.keywords, [])

    def test_무효_모드는_변환_발행보다_먼저_return(self):
        cb = self._func('color_callback')
        gate = self._call_lines(cb, name='detection_disabled')
        self.assertEqual(len(gate), 1)
        conv = self._call_lines(cb, attr='imgmsg_to_cv2')
        proc = self._call_lines(cb, attr='process_image')
        pubs = self._call_lines(cb, attr='publish') + self._call_lines(cb, name='publish')
        self.assertTrue(conv and proc and pubs)
        self.assertLess(gate[0], min(conv + proc + pubs))
        # detection_disabled 가 참이면 바로 return 한다
        ifs = [n for n in ast.walk(cb) if isinstance(n, ast.If)
               and isinstance(n.test, ast.Call) and isinstance(n.test.func, ast.Name)
               and n.test.func.id == 'detection_disabled']
        self.assertEqual(len(ifs), 1)
        self.assertIsInstance(ifs[0].body[-1], ast.Return)
        self.assertIsNone(ifs[0].body[-1].value)
        # 그 분기 안에서는 발행하지 않는다(10000 센티널도 없음) — 주기 ERROR 만 낸다.
        branch = ast.Module(body=ifs[0].body, type_ignores=[])
        self.assertEqual(self._call_lines(branch, attr='publish'), [])
        self.assertEqual(self._call_lines(branch, name='publish'), [])

    def test_무효_모드는_log_due_로_주기_ERROR(self):
        cb = self._func('color_callback')
        ifs = [n for n in ast.walk(cb) if isinstance(n, ast.If)
               and isinstance(n.test, ast.Call) and isinstance(n.test.func, ast.Name)
               and n.test.func.id == 'detection_disabled']
        self.assertEqual(len(ifs), 1)
        branch = ast.Module(body=ifs[0].body, type_ignores=[])
        self.assertEqual(len(self._call_lines(branch, name='log_due')), 1)
        self.assertEqual(len(self._call_lines(branch, attr='error')), 1)
        for bad in ('warn', 'warning', 'info'):
            self.assertEqual(self._call_lines(branch, attr=bad), [], bad)
        seg = ast.get_source_segment(self.src, ifs[0])
        self.assertIn('turn 목표 색 무효(mode ', seg)
        self.assertIn('turn_color_mode', seg)
        self.assertIn('_last_color_err_log = now_m', seg)
        # 시각은 모드 게이트가 쓴 time.monotonic() 값(now_m) — time.time() 금지(1-5)
        self.assertNotIn('time.time', self.src)

    def test_주기_로그_시각은_구독보다_먼저_None_으로(self):
        init = self._init()
        inits = [n.lineno for n in ast.walk(init) if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Attribute) and t.attr == '_last_color_err_log'
                         for t in n.targets)
                 and isinstance(n.value, ast.Constant) and n.value.value is None]
        subs = [n.lineno for n in ast.walk(init)
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'create_subscription']
        self.assertEqual(len(inits), 1)
        self.assertLess(inits[0], min(subs))

    def test_process_image_는_target_colors_for_를_쓴다(self):
        # wp_color_map 을 직접 `in` 판정하면 빈 목록 처리가 노드·로직 사이에서 갈라진다.
        proc = self._func('process_image')
        self.assertEqual(len(self._call_lines(proc, name='target_colors_for')), 1)
        for n in ast.walk(proc):
            if isinstance(n, ast.Compare) and any(isinstance(o, ast.In) for o in n.ops):
                for c in n.comparators:
                    self.assertFalse(isinstance(c, ast.Attribute) and c.attr == 'wp_color_map',
                                     "process_image 가 wp_color_map 을 직접 `in` 판정한다")

    def test_HSV_로더가_같은_색_튜플을_쓴다(self):
        self.assertIn('TURN_COLORS,', self.src)
        self.assertNotIn('("red", "green", "white")', self.src)


if __name__ == '__main__':
    unittest.main()
