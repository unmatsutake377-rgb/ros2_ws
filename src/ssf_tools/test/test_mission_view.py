#!/usr/bin/env python3
"""mission_view 순수 로직 테스트 — ROS·장비 없이 돈다."""

import unittest

from ssf_tools.mission_view import (
    boat_mode_name, fmt_bool_alarm, fmt_dist, fmt_secs,
    mission_name, render, rtk_label,
)


class TestMissionName(unittest.TestCase):

    def test_실제_웨이포인트_모드_전부(self):
        # CLAUDE.md 3-6: 실제로 나오는 값은 {0,1,2,3,5,7,8} 뿐
        for m in (0, 1, 2, 3, 5, 7, 8):
            self.assertNotIn("미정의", mission_name(m))

    def test_도킹은_7이다(self):
        # 🚨 작년에 ship_dock 이 9 로 선언돼 1년간 침묵했다(CLAUDE.md 3-1).
        #    화면에도 7 이 도킹으로 보여야 그 사고를 눈으로 잡는다.
        self.assertEqual(mission_name(7), "도킹")

    def test_모르는_모드는_숨기지_않는다(self):
        out = mission_name(9)
        self.assertIn("9", out)
        self.assertIn("미정의", out)

    def test_없으면_대시(self):
        self.assertEqual(mission_name(None), "—")


class TestBoatMode(unittest.TestCase):

    def test_세_모드(self):
        self.assertIn("대기", boat_mode_name(0))
        self.assertIn("수동", boat_mode_name(1))
        self.assertIn("자율", boat_mode_name(2))

    def test_없으면_대시(self):
        self.assertEqual(boat_mode_name(None), "—")

    def test_모르는_값은_경고(self):
        self.assertIn("🚨", boat_mode_name(5))


class TestRtkLabel(unittest.TestCase):

    def test_fixed_실측값(self):
        # 2026-08-07 야외 실측: ±0.015m
        self.assertIn("FIXED", rtk_label(0.015))

    def test_float_실측값(self):
        # 같은 세션에서 수렴 전 ±0.10m
        self.assertIn("float", rtk_label(0.10))

    def test_단독측위(self):
        self.assertIn("단독", rtk_label(3.0))

    def test_경계값은_좋은쪽으로(self):
        self.assertIn("FIXED", rtk_label(0.05))
        self.assertIn("float", rtk_label(0.50))

    def test_없으면_대시(self):
        self.assertEqual(rtk_label(None), "—")


class TestFormatters(unittest.TestCase):

    def test_거리(self):
        self.assertEqual(fmt_dist(12.34), "12.3 m")
        self.assertEqual(fmt_dist(None), "—")

    def test_초(self):
        self.assertEqual(fmt_secs(59.6), "60 s")
        self.assertEqual(fmt_secs(None), "—")

    def test_불리언_경보(self):
        self.assertEqual(fmt_bool_alarm(True, "나쁨", "정상"), "나쁨")
        self.assertEqual(fmt_bool_alarm(False, "나쁨", "정상"), "정상")
        self.assertEqual(fmt_bool_alarm(None, "나쁨", "정상"), "—")


class TestRender(unittest.TestCase):

    def test_아무것도_없어도_안_죽는다(self):
        # 부팅 직후엔 모든 값이 None 이다. 여기서 예외가 나면 화면이 아예 안 뜬다.
        out = render({})
        self.assertIn("미션 모니터", out)

    def test_브릿지_없으면_모드를_비워두지_않고_이유를_적는다(self):
        out = render({"bridge_seen": False})
        self.assertIn("브릿지 미연결", out)

    def test_받다가_끊긴_것은_미연결과_구분한다(self):
        # 🚨 '아직 안 옴' 과 '받다가 끊김' 은 다른 사건이다.
        #    끊김은 자율 명령 중단 → 펌웨어 워치독 → 곧 중립을 뜻한다.
        out = render({"bridge_seen": True, "boat_mode": None})
        self.assertIn("끊김", out)
        self.assertNotIn("미연결", out)

    def test_브릿지_있으면_모드를_보여준다(self):
        out = render({"bridge_seen": True, "boat_mode": 2, "boat_id": 1})
        self.assertIn("자율", out)

    def test_워치독_발동이_보인다(self):
        out = render({"bridge_seen": True, "boat_mode": 2, "watchdog": True})
        self.assertIn("끊김", out)

    def test_비상정지는_눌렸을_때만_줄이_생긴다(self):
        # boat_mode 를 같이 넣어야 한다 — 비상정지는 모드와 **같은 상태 줄**에서 온다.
        # 브릿지가 끊기면 비상정지 여부도 모르는 게 맞다(그래서 줄 자체가 안 나온다).
        off = render({"bridge_seen": True, "boat_mode": 1, "estop": False})
        on = render({"bridge_seen": True, "boat_mode": 1, "estop": True})
        self.assertNotIn("비상정지", off)
        self.assertIn("비상정지", on)

    def test_브릿지_끊기면_비상정지도_모른다(self):
        out = render({"bridge_seen": True, "boat_mode": None, "estop": True})
        self.assertNotIn("비상정지", out)

    def test_전체_정상_화면(self):
        out = render({
            "bridge_seen": True, "boat_mode": 2, "boat_id": 1,
            "watchdog": False, "estop": False,
            "wp_mode": 7, "goal_dist": 4.2, "wp_remain": 38.0,
            "rtk_sigma": 0.015, "failsafe": 0, "gates": 2, "health_ok": True,
        })
        self.assertIn("도킹", out)
        self.assertIn("FIXED", out)
        self.assertIn("자율", out)
        self.assertIn("✅ true", out)

    def test_health_false_는_눈에_띈다(self):
        out = render({"health_ok": False})
        self.assertIn("🚨", out)

    def test_페일세이프_정지(self):
        out = render({"failsafe": 2})
        self.assertIn("정지", out)


if __name__ == '__main__':
    unittest.main()
