#!/usr/bin/env bash
# cam_stream.sh — 배 노트북에서 D455 컬러 영상을 물가 노트북으로 쏜다 (송신측).
#
# 왜:
#   수동 모드에서 조종 위치가 배에서 멀다. 조종자가 배 시점 영상을 봐야 한다.
#   ROS2 토픽은 Tailscale/서로 다른 랜을 못 넘는다(SSH환경구축_프롬프트.md).
#   → ROS 와 무관한 **단방향 UDP/RTP(H.264)** 로 보낸다. 유니캐스트라 VPN 도 넘는다.
#
# 설계 원칙 (저장소 공통):
#   · 모르면 입을 다문다 → 영상에 **배 시각을 박는다**(clockoverlay).
#     화면이 멈추면 시계도 멈춘다 = "묵은 프레임" 을 "지금" 으로 읽는 사고를 막는다.
#   · 제어 경로와 완전 분리. 이 스크립트가 죽어도 RC 수동 조종은 영향 0 (펌웨어 단독).
#   · 실패 시 조용히 안 죽는다 — 파이프라인이 끊기면 3초 후 자동 재시작(루프).
#
# 사용:
#   ./tools/cam_stream.sh <물가노트북IP> [v4l2|ros|oak] [폭] [높이] [fps] [kbps]
#   ./tools/cam_stream.sh 100.101.102.103            # D455 수동 모드(카메라 노드 안 뜸): v4l2 직접
#   ./tools/cam_stream.sh 100.101.102.103 ros        # 카메라 노드가 떠 있을 때(D455/OAK 공통): 토픽 경유
#   ./tools/cam_stream.sh 100.101.102.103 oak        # OAK 온보드 인코더 직접 (CPU 0, 시각 각인 없음)
#   ./tools/cam_stream.sh 100.101.102.103 v4l2 640 480 30 1500
#   CAM_TOPIC=/oak/rgb/image_raw ./tools/cam_stream.sh 100.101.102.103 ros   # OAK(depthai-ros) 토픽
#
# 카메라별 (🚨 OAK-1 PoE(확정 09-15) 로 바뀌면 v4l2 모드는 못 쓴다 — PoE 는 이더넷 장치라 /dev/video 가 없다):
#   | 카메라          | 수동(카메라 노드 없음)      | 자율(카메라 노드 있음)              |
#   | RealSense D455  | v4l2                       | ros                                 |
#   | OAK-1 PoE       | ros (depthai 노드만 띄움) 또는 oak | ros (CAM_TOPIC=/oak/rgb/image_raw ⚠️ 도착일 확인) |
#
# ⚠️ v4l2 모드는 카메라 장치를 직접 여는 것이라 realsense2_camera 노드가 떠 있으면
#    "Device busy" 로 실패한다. 그때는 ros 모드.
#
# 의존 (배 노트북, Ubuntu 22.04):
#   sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
#                    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
#                    v4l-utils python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0
#   (ros 모드는 python3-gi 필요. 인코더는 x264enc = CPU. VAAPI 는 선택, 아래 참고)
#
# 롤백: 이 스크립트를 Ctrl-C 로 끝내면 끝. 시스템 설정을 바꾸지 않는다.

set -uo pipefail

HOST="${1:-}"
MODE="${2:-v4l2}"
W="${3:-640}"
H="${4:-480}"
FPS="${5:-30}"
KBPS="${6:-1500}"      # ⚠️ 실측 필요: 5GHz 랜이면 3000 까지, LTE 면 800~1500 권장
PORT="${CAM_PORT:-5600}"
DEV="${CAM_DEV:-}"     # 비우면 자동 탐색 (D455 컬러 = YUYV 포맷을 내는 노드)
TOPIC="${CAM_TOPIC:-/camera/camera/color/image_raw}"   # ros 모드 토픽. OAK 는 CAM_TOPIC 으로 덮어씀
OAK_IP="${CAM_OAK_IP:-}"                                # oak 모드 카메라 고정 IP (비우면 자동 탐색)

if [ -z "$HOST" ]; then
  echo "❌ 사용: $0 <물가노트북IP> [v4l2|ros|oak] [폭] [높이] [fps] [kbps]" >&2
  exit 1
fi
command -v gst-launch-1.0 >/dev/null 2>&1 || { echo "❌ gstreamer 없음. 헤더의 apt 줄 참고." >&2; exit 1; }

# ── D455 컬러 노드 자동 탐색 ────────────────────────────────────────────────
# D455 는 /dev/video 노드를 여러 개 낸다(뎁스·IR·컬러·메타데이터). 컬러만 YUYV 를 낸다.
find_color_dev() {
  command -v v4l2-ctl >/dev/null 2>&1 || { echo ""; return; }
  for d in /dev/video*; do
    if v4l2-ctl -d "$d" --list-formats 2>/dev/null | grep -q "YUYV"; then
      echo "$d"; return
    fi
  done
  echo ""
}

# ── 공통 후단: 인코딩 → RTP → UDP ──────────────────────────────────────────
# x264enc: tune=zerolatency speed-preset=ultrafast → 인코딩 지연 1프레임 미만.
#   key-int-max=FPS/2 → 0.5초마다 키프레임. 벤치(cam_bench) 결과: 유실 1~2% 링크에서 1초 간격은
#   유실 뒤 묵은 영역이 최대 1.6~3s 남았고, 0.5초로 줄이니 p99 300ms 이하로 떨어졌다(설계문서 §7-B).
#   bitrate 는 kbps. 640x480@30 에서 1500 이면 조종하기엔 충분히 선명하다(⚠️ 실측).
# clockoverlay: 배 노트북 시각을 좌상단에 박는다 — 프리즈 판별용 (헤더 '왜' 참고).
ENC="clockoverlay time-format=\"%H:%M:%S\" halignment=left valignment=top font-desc=\"Sans 18\" \
 ! videoconvert ! video/x-raw,format=I420 \
 ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=${KBPS} key-int-max=$((FPS/2)) bframes=0 \
 ! rtph264pay config-interval=1 pt=96 mtu=1200 \
 ! udpsink host=${HOST} port=${PORT} sync=false async=false"
# mtu=1200: Tailscale(WireGuard) 헤더 몫을 빼서 IP 단편화를 피한다.

# (선택) 인텔 내장그래픽이면 VAAPI 로 CPU 를 아낄 수 있다 — 검증 후에만 바꿀 것:
#   gstreamer1.0-vaapi 설치 후 x264enc 줄을
#   "vaapih264enc rate-control=cbr bitrate=${KBPS} keyframe-period=$((FPS/2))" 로 교체.
#   ⚠️ 미검증. IdeaPad Slim 3 의 CPU/GPU 종류부터 확인(lscpu, vainfo).

echo "=== 카메라 송신 ==="
echo "  대상    : ${HOST}:${PORT} (UDP/RTP H.264)"
echo "  모드    : ${MODE}  ${W}x${H}@${FPS}  ${KBPS}kbps"

while true; do
  case "$MODE" in
    v4l2)
      [ -n "$DEV" ] || DEV="$(find_color_dev)"
      if [ -z "$DEV" ]; then
        echo "⚠️ YUYV 를 내는 /dev/video 노드가 없다. 카메라 USB 확인. 3초 후 재시도." >&2
        sleep 3; continue
      fi
      echo "  장치    : ${DEV}"
      # v4l2src io-mode=2(mmap). do-timestamp 로 캡처 시각 기준 타임스탬프.
      eval gst-launch-1.0 -q v4l2src device="${DEV}" io-mode=2 do-timestamp=true \
        ! "video/x-raw,format=YUY2,width=${W},height=${H},framerate=${FPS}/1" \
        ! "$ENC"
      ;;
    ros)
      # realsense 노드가 장치를 잡고 있을 때: /camera/camera/color/image_raw → appsrc
      SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
      python3 "${SCRIPT_DIR}/cam_stream_ros.py" --host "${HOST}" --port "${PORT}" \
        --kbps "${KBPS}" --fps "${FPS}" --topic "${TOPIC}"
      ;;
    oak)
      # OAK-1 PoE 온보드 H.264 인코더 직접. ⚠️ 시각 각인 없음 — 헤더 표 참고. ⚠️ 실물 미검증.
      SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
      python3 "${SCRIPT_DIR}/cam_stream_oak.py" --host "${HOST}" --port "${PORT}" \
        --kbps "${KBPS}" --fps "${FPS}" --w "${W}" --h "${H}" ${OAK_IP:+--ip "${OAK_IP}"}
      ;;
    *)
      echo "❌ 모드는 v4l2 / ros / oak" >&2; exit 1 ;;
  esac
  echo "⚠️ 송신 파이프라인 종료(코드 $?). 3초 후 재시작. 끝내려면 Ctrl-C." >&2
  sleep 3
done
