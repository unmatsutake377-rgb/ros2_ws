# tools/cad — 마스트·배치도 생성 스크립트

- `mast.py` — 카메라 마스트 파라메트릭 CAD (CadQuery). `pip install cadquery` 후 `python3 tools/cad/mast.py` → `tools/cad/out/mast_*.step|stl`
  (out/ 은 git 에 안 올림. 프린트용 STL 확정본은 `docs/전달용/마스트_stl/` 에 복사해 둔다)
  **[v2 2026-09-16]** OAK 위(−11°)/D455 아래(−15°), 베이스 20mm + IMU 포켓·뚜껑(`mast_imu_lid`), 전고 220 검증 출력. 근거 `docs/전달용/마스트_요구사항.md` §2-5·2-6.

## Fusion 360 에서 여는 법
1. `docs/전달용/마스트_stl/mast_assembly_v2.step` 을 Fusion 360 **파일 > 열기 > 내 컴퓨터에서 열기**(또는 데이터 패널에 업로드). 부품별 바디(base, seg1, cap, imu_lid)와 카메라·IMU 더미(OAK_dummy, D455_dummy, IMU_dummy)가 컴포넌트로 들어온다.
2. 치수 바꾸려면 Fusion 에서 직접 편집하지 말고 **`mast.py` 파라미터를 고쳐 재생성**(파라메트릭 원본은 스크립트). Fusion 은 검토·간섭 확인·도면용.
3. 프린트는 `mast_*.stl` 을 슬라이서로. 베이스는 바닥면 아래로, 마디는 세워서(판·선반 밑 서포트 필요).
4. 실측 후 갱신 목록: IMU_L/W/H(iAHRS 케이스 포함), OAK_* (도착 시), BASE_HOLE_PITCH(가로대), 틸트 2개(물 위 시험).
- `layout.py` — 갑판 위 박스 구성 시절의 배치도 (보관용)
- `concept_recessed.py` — **현행** 구상도: 갑판 덮은 오픈 헐, 박스는 갑판 아래 → `docs/전달용/배치도/concept_deck.png`
- 선체 STL 분석(거널 폭·오픈 헐 판정)은 2026-09-15 세션에서 trimesh 로 수행. 선체 STL 자체(4MB)는 저장소에 넣지 않았다 — 선체 담당 보관.

요구사항·결정 근거: `docs/전달용/하드웨어_배치_요구사항.md`
