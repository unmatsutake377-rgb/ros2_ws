#!/usr/bin/env bash
# cam_view.sh — 물가 노트북에서 배 영상을 띄운다 (수신측).
#
# 왜: cam_stream.sh 의 짝. ROS 불필요 — gstreamer 만 있으면 Ubuntu/Mac 어디서든 뜬다.
#
# 사용:
#   ./tools/cam_view.sh            # 5600 포트 대기
#   CAM_PORT=5600 ./tools/cam_view.sh
#
# 설계:
#   · sync=false        → 프레임을 받는 즉시 그린다(재생 시계에 맞춰 기다리지 않음). 지연 최소.
#   · jitterbuffer 30ms → 무선 지터 흡수. 늘리면 부드럽지만 지연이 는다(⚠️ 실측으로 조정).
#   · 화면 좌상단의 시계가 **배 시각**이다. 시계가 멈추면 영상이 멈춘 것 — 그 프레임을 믿지 말 것.
#   · 수신 노트북 자체 시각도 우하단에 박는다. 두 시계 차이 ≈ 전송 지연(양쪽 NTP 맞으면).
#
# 의존 (Ubuntu): gstreamer1.0-tools gstreamer1.0-plugins-{base,good,bad,ugly} gstreamer1.0-libav
#      (Mac)   : brew install gstreamer  (autovideosink 대신 osxvideosink 가 잡힌다)
#
# 방화벽: 수신측에서 UDP ${PORT} 인바운드가 열려 있어야 한다.
#   sudo ufw allow ${PORT}/udp   (ufw 쓰는 경우)

set -uo pipefail
PORT="${CAM_PORT:-5600}"
LAT="${CAM_JITTER_MS:-30}"

command -v gst-launch-1.0 >/dev/null 2>&1 || { echo "❌ gstreamer 없음. 헤더 참고." >&2; exit 1; }

echo "=== 카메라 수신 === UDP ${PORT} 대기 (지터버퍼 ${LAT}ms). 끝내려면 창 닫기/Ctrl-C."
echo "  ⚠️ 화면 좌상단 시계 = 배 시각. 멈추면 영상이 멈춘 것."

while true; do
  gst-launch-1.0 -q udpsrc port="${PORT}" caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
    ! rtpjitterbuffer latency="${LAT}" drop-on-latency=true \
    ! rtph264depay ! h264parse ! avdec_h264 max-threads=2 \
    ! videoconvert \
    ! clockoverlay time-format="%H:%M:%S" halignment=right valignment=bottom font-desc="Sans 14" \
    ! autovideosink sync=false
  echo "⚠️ 수신 파이프라인 종료. 3초 후 재시작." >&2
  sleep 3
done
