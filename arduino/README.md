# arduino — 회로(Arduino Due) 펌웨어

배에 탑재되는 Arduino Due 코드. 설계 근거와 결정 사항은 `docs/펌웨어_설계문서.md` 참고.

- `ssf_boat/ssf_boat.ino` — 펌웨어 본체 (Arduino IDE로 열어서 업로드)
- 업로드: Arduino IDE → 보드 "Arduino Due (Programming Port)" → Programming 포트에 USB 연결 → 업로드
- 노트북과의 토픽 계약: `Motor_run` 구독(Int32, pwm_r*10000+pwm_l, 1500=중립), `/firmware_status` 발행(Int32MultiArray, 10Hz)
- 배 A/B 구분은 코드가 아니라 **배에 달린 ID 핀(DIP 스위치)** 으로 자동 인식 — 펌웨어는 두 배 공용 1벌
