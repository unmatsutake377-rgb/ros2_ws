#!/usr/bin/env python3
"""status_parser 순수 로직 테스트 — 아두이노 없이 돈다.

⚠️ 이 테스트가 다 통과해도 **실물 검증이 아니다.**
   작년 페일세이프 버그(CLAUDE.md 5장)가 순수 로직 테스트 27개를 전부 통과한 채
   실기에서 노드를 죽였다. 여기서 보는 건 '줄을 올바로 해석하는가' 뿐이다.
"""

import unittest

from ssf_bridge.status_parser import (
    BOAT_A, BOAT_B, BOAT_FAULT,
    MODE_AUTO, MODE_MANUAL, MODE_WAIT,
    boat_name, clamp_pwm, decode_motor_run, format_command,
    mode_name, parse_status_line,
)


class TestParseStatusLine(unittest.TestCase):

    def test_정상_수동(self):
        r = parse_status_line("S,1,0,0,0,1500,1500,1500,1500")
        self.assertEqual(r["mode"], MODE_MANUAL)
        self.assertFalse(r["watchdog"])
        self.assertEqual(r["boat_id"], BOAT_A)
        self.assertFalse(r["estop"])
        self.assertEqual(r["outputs"], [1500, 1500, 1500, 1500])

    def test_정상_자율_전진(self):
        r = parse_status_line("S,2,0,1,0,1400,1400,1400,1400")
        self.assertEqual(r["mode"], MODE_AUTO)
        self.assertEqual(r["boat_id"], BOAT_B)

    def test_워치독_발동(self):
        r = parse_status_line("S,2,1,0,0,1500,1500,1500,1500")
        self.assertTrue(r["watchdog"])

    def test_비상정지(self):
        r = parse_status_line("S,1,0,0,1,1500,1500,1500,1500")
        self.assertTrue(r["estop"])

    def test_배ID_고장(self):
        r = parse_status_line("S,0,0,2,0,1500,1500,1500,1500")
        self.assertEqual(r["boat_id"], BOAT_FAULT)

    def test_줄바꿈_공백_허용(self):
        self.assertIsNotNone(parse_status_line("  S,1,0,0,0,1500,1500,1500,1500\r\n"))

    # ── 아래는 전부 None 이어야 한다 ('모르면 입을 다문다') ──

    def test_거부_빈줄(self):
        self.assertIsNone(parse_status_line(""))

    def test_거부_None(self):
        self.assertIsNone(parse_status_line(None))

    def test_거부_비문자열(self):
        self.assertIsNone(parse_status_line(12345))

    def test_거부_접두어_다름(self):
        # 명령 에코나 디버그 줄이 섞여 들어오는 경우
        self.assertIsNone(parse_status_line("L1500,R1500"))

    def test_거부_필드_부족(self):
        self.assertIsNone(parse_status_line("S,1,0,0,0,1500,1500,1500"))

    def test_거부_필드_초과(self):
        self.assertIsNone(parse_status_line("S,1,0,0,0,1500,1500,1500,1500,1500"))

    def test_거부_숫자아님(self):
        self.assertIsNone(parse_status_line("S,1,0,0,0,1500,1500,15x0,1500"))

    def test_거부_모드_범위밖(self):
        self.assertIsNone(parse_status_line("S,7,0,0,0,1500,1500,1500,1500"))

    def test_거부_배ID_범위밖(self):
        self.assertIsNone(parse_status_line("S,1,0,9,0,1500,1500,1500,1500"))

    def test_거부_워치독_불리언아님(self):
        self.assertIsNone(parse_status_line("S,1,5,0,0,1500,1500,1500,1500"))

    def test_거부_PWM_범위밖_상한(self):
        self.assertIsNone(parse_status_line("S,1,0,0,0,1500,1500,1500,2500"))

    def test_거부_PWM_범위밖_하한(self):
        self.assertIsNone(parse_status_line("S,1,0,0,0,900,1500,1500,1500"))

    def test_거부_잘린줄_노이즈(self):
        # 시리얼 노이즈로 앞부분이 잘린 실제 형태
        self.assertIsNone(parse_status_line("1,0,0,0,1500,1500,1500,1500"))


class TestNames(unittest.TestCase):

    def test_모드_이름(self):
        self.assertEqual(mode_name(MODE_WAIT), "대기")
        self.assertEqual(mode_name(MODE_MANUAL), "수동(RC)")
        self.assertEqual(mode_name(MODE_AUTO), "자율(AUTO)")

    def test_모르는_모드는_번호를_드러낸다(self):
        # 숨기면 "왜 이상하지" 를 추적할 수 없다
        self.assertIn("9", mode_name(9))

    def test_배_이름(self):
        self.assertEqual(boat_name(BOAT_A), "A")
        self.assertEqual(boat_name(BOAT_FAULT), "FAULT")


class TestMotorRunCodec(unittest.TestCase):
    """🚨 motor_control 인코딩과의 왕복 일치 — 어긋나면 배가 엉뚱하게 움직인다."""

    def test_중립(self):
        self.assertEqual(decode_motor_run(1500 * 10000 + 1500), (1500, 1500))

    def test_좌우_구분(self):
        # pwm_r=1600, pwm_l=1400
        self.assertEqual(decode_motor_run(1600 * 10000 + 1400), (1400, 1600))

    def test_왕복_전_범위(self):
        for l in range(1000, 2001, 100):
            for r in range(1000, 2001, 100):
                self.assertEqual(decode_motor_run(r * 10000 + l), (l, r))

    def test_범위밖은_잘린다(self):
        self.assertEqual(clamp_pwm(999), 1000)
        self.assertEqual(clamp_pwm(2001), 2000)

    def test_명령_형식(self):
        self.assertEqual(format_command(1400, 1600), "L1400,R1600\n")

    def test_명령_형식이_범위를_강제한다(self):
        self.assertEqual(format_command(500, 9999), "L1000,R2000\n")


if __name__ == '__main__':
    unittest.main()
