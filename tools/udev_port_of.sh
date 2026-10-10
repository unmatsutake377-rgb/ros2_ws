#!/usr/bin/env bash
# 꽂혀 있는 시리얼 장치마다 udev 규칙에 쓸 값을 찍는다 — 2번배 노트북 규칙 작성용.
#
# 🚨 왜 필요한가 (2026-10-07): 싸구려 CP2102 어댑터는 시리얼이 전부 `0001` 이다.
#   2번배는 IMU 어댑터도, 라이다 어댑터도 0001 이라 ATTRS{serial} 로는 못 가른다.
#   남은 수단은 **물리 포트(KERNELS)** 뿐이다. 이 스크립트가 그 값을 장치별로 찍어준다.
#
# 쓰는 법:  장치를 하나씩 꽂을 때마다 실행해서 어느 줄이 새로 생겼는지 본다.
#   bash tools/udev_port_of.sh
# 그 KERNELS 값을 docs/절차/udev_2번배_템플릿.rules 의 빈칸에 넣는다.
#
# ⚠️ KERNELS 는 "허브의 그 구멍" 이다. 다른 구멍에 옮겨 꽂으면 이름이 바뀐다.
#    배선도에 어느 장치가 어느 구멍인지 적어둘 것.

for p in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$p" ] || continue
  P=$(udevadm info -q property -n "$p" 2>/dev/null)
  vid=$(echo "$P" | sed -n 's/^ID_VENDOR_ID=//p'); pid=$(echo "$P" | sed -n 's/^ID_MODEL_ID=//p')
  ser=$(echo "$P" | sed -n 's/^ID_SERIAL_SHORT=//p'); mdl=$(echo "$P" | sed -n 's/^ID_MODEL=//p')
  # KERNELS: tty 의 부모 USB 인터페이스 → 그 위 장치. udevadm -a 의 첫 "3-1.4.2" 꼴.
  ker=$(udevadm info -a -n "$p" 2>/dev/null | grep -oE 'KERNELS=="[0-9]+-[0-9.]+"' | grep -v ':' | head -1)
  uniq="공장값(0001) — serial 로 못 가름"
  [ -n "$ser" ] && [ "$ser" != "0001" ] && uniq="유일 — ATTRS{serial} 로 가를 수 있음"
  printf '%s\n' "$p"
  printf '   vid:pid  %s:%s   model  %s\n' "$vid" "$pid" "$mdl"
  printf '   serial   %-36s %s\n' "${ser:-(없음)}" "$uniq"
  printf '   %s   ← 규칙에 쓸 물리 포트\n\n' "${ker:-KERNELS 못 찾음}"
done
echo '규칙 템플릿: docs/절차/udev_2번배_템플릿.rules'
