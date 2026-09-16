# 마스트 STL — v2 (2026-09-16)

프린트: `mast_base.stl`(IMU 포켓·칼라) / `mast_seg1.stl`(각관 + D455 선반) / `mast_cap.stl`(OAK 판) / `mast_imu_lid.stl`(뚜껑, 나일론 M3×4).
Fusion 360 확인: `mast_assembly_v2.step` (D455·OAK·IMU 더미 포함) → Inspect > Interference.

⚠️ **아직 프린트하지 말 것** — `IMU_L/W/H` 가 가정값(50×50×20). iAHRS 케이스 실측 후 `tools/cad/mast.py` 만 고쳐 다시 뽑는다(과제 B10).
재질 PETG/ASA(PLA ✗), 밝은 색, 인필 ≥40%, 벽 ≥4. 방향: base·cap 은 그대로, seg1 은 관을 세워서(선반은 서포트).

`v1_구버전/` = D455 위·OAK 아래 시절(09-15). 쓰지 않음, 기록용.
