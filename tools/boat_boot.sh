#!/usr/bin/env bash
# boat_boot.sh — launch 전에 노트북을 '경기 모드'로 고정한다.
#
# 왜 (CLAUDE.md 3-2 표현 원칙, 검증 피드백 §2-③):
#   ⚠️ 우리는 제어 지연을 측정한 적이 없다. "전원 관리가 제어 지연의 원인이다" 는 **미검증 가설**이다.
#   그래도 이 조치를 하는 이유는 원인 단정이 아니라 **싸고 무해한 보험**이기 때문이다:
#     · USB autosuspend 가 시리얼 장치(LiDAR/IMU/GPS)를 끊는 것은 **실재하는 리눅스 이슈**다.
#     · CPU 거버너가 절전으로 내려가면 제어 주기가 흔들릴 수 있다(알려진 요인, 원인 규명은 별도).
#     · 화면 잠금/서스펜드가 경기 중 걸리면 그대로 사고다.
#   → "제어 지연을 유발할 수 있는 알려진 요인이라 미리 차단한다. 원인 규명은 별도 측정."
#
# 사용:
#   sudo ./tools/boat_boot.sh          # 전체 적용 (거버너·USB·절전 전부)
#   ./tools/boat_boot.sh --check       # 지금 상태만 출력 (변경 없음, sudo 불필요)
#
# ⚠️ Ubuntu 전용. root 가 필요한 항목이 있어 sudo 로 실행한다.
#    이 스크립트는 **되돌릴 수 있는 런타임 설정만** 만진다(재부팅하면 원상복구).
#    영구 설정(GRUB, systemd 서비스)은 건드리지 않는다 — 실수의 대가가 크다.

set -uo pipefail

CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

ok(){ printf '  \033[32m✓\033[0m %s\n' "$*"; }
warn(){ printf '  \033[33m!\033[0m %s\n' "$*"; }
skip(){ printf '  \033[90m·\033[0m %s\n' "$*"; }

need_root(){
  if [ "$(id -u)" -ne 0 ] && [ "$CHECK_ONLY" -eq 0 ]; then
    echo "❌ root 가 필요하다. 'sudo $0' 로 실행하거나 --check 로 상태만 봐라." >&2
    exit 1
  fi
}

# ─────────────────────────────────────────── 1) CPU 거버너
cpu_governor(){
  echo "[1] CPU 거버너 → performance"
  local any=0
  for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    [ -e "$g" ] || continue
    any=1
    local cur; cur=$(cat "$g" 2>/dev/null)
    if [ "$CHECK_ONLY" -eq 1 ]; then
      skip "$(dirname "$(dirname "$g")" | xargs basename): $cur"
      continue
    fi
    if echo performance > "$g" 2>/dev/null; then :; else warn "$g 쓰기 실패"; fi
  done
  [ "$any" -eq 0 ] && { warn "cpufreq 인터페이스 없음 (VM/도커?) — 건너뜀"; return; }
  [ "$CHECK_ONLY" -eq 0 ] && ok "전 코어 performance"
}

# ─────────────────────────────────────────── 2) USB autosuspend
# 🚨 시리얼 장치(LiDAR/IMU/GPS)가 autosuspend 로 끊기는 실재 이슈 차단.
usb_autosuspend(){
  echo "[2] USB autosuspend 비활성 (시리얼 장치 보호)"
  # 전역 스위치
  local gp=/sys/module/usbcore/parameters/autosuspend
  if [ -e "$gp" ]; then
    if [ "$CHECK_ONLY" -eq 1 ]; then
      skip "usbcore.autosuspend = $(cat "$gp") (-1 이면 꺼짐)"
    else
      echo -1 > "$gp" 2>/dev/null && ok "usbcore.autosuspend = -1" || warn "$gp 쓰기 실패"
    fi
  fi
  # 장치별 power/control = on
  local n=0
  for c in /sys/bus/usb/devices/*/power/control; do
    [ -e "$c" ] || continue
    if [ "$CHECK_ONLY" -eq 1 ]; then
      [ "$(cat "$c")" = "auto" ] && n=$((n+1))
    else
      echo on > "$c" 2>/dev/null && n=$((n+1))
    fi
  done
  if [ "$CHECK_ONLY" -eq 1 ]; then
    [ "$n" -gt 0 ] && warn "$n 개 장치가 아직 auto (절전 대상)" || ok "장치별 절전 없음"
  else
    ok "$n 개 USB 장치 power/control = on"
  fi
}

# ─────────────────────────────────────────── 3) 화면 절전/서스펜드
# 경기 중 화면 잠금·서스펜드가 걸리면 그대로 사고. 런타임만 끈다.
screen_sleep(){
  echo "[3] 화면 절전 / 서스펜드 비활성 (런타임)"
  if [ "$CHECK_ONLY" -eq 1 ]; then
    if command -v systemctl >/dev/null; then
      local m; m=$(systemctl is-enabled sleep.target 2>/dev/null || echo "?")
      skip "sleep.target: $m"
    fi
    return
  fi
  # systemd 서스펜드 억제 (재부팅 시 원복 — mask 아님)
  if command -v systemctl >/dev/null; then
    systemctl stop  sleep.target suspend.target hibernate.target 2>/dev/null || true
    ok "sleep/suspend/hibernate target stop"
  fi
  # X11 이 있으면 화면보호기·DPMS 끄기 (SSH 헤드리스면 DISPLAY 없음 → 건너뜀)
  if [ -n "${DISPLAY:-}" ] && command -v xset >/dev/null; then
    xset s off -dpms 2>/dev/null && ok "xset s off -dpms" || warn "xset 실패"
  else
    skip "DISPLAY 없음(헤드리스) — xset 건너뜀"
  fi
}

# ─────────────────────────────────────────── 4) 전원 상태 알림 (변경 아님)
power_note(){
  echo "[4] 전원 상태"
  local ac=/sys/class/power_supply/AC*/online
  for a in $ac; do
    [ -e "$a" ] || continue
    [ "$(cat "$a")" = "1" ] && ok "AC 연결됨" || warn "🔋 배터리로 구동 중 — 경기 전 AC 연결 권장"
    return
  done
  skip "AC 어댑터 정보 없음"
}

echo "=== boat_boot $([ "$CHECK_ONLY" -eq 1 ] && echo '(상태 확인만)') ==="
need_root
cpu_governor
usb_autosuspend
screen_sleep
power_note
echo
if [ "$CHECK_ONLY" -eq 1 ]; then
  echo "상태 확인 완료. 적용하려면: sudo $0"
else
  echo "✅ 경기 모드 적용 완료. 전부 런타임 설정이라 재부팅하면 원복된다."
  echo "   launch 전에 매번 실행할 것 (README 참고)."
fi
