# tools/cad — 마스트·배치도 생성 스크립트

- `mast.py` — 카메라 마스트 파라메트릭 CAD (CadQuery). `pip install cadquery` 후 `python3 tools/cad/mast.py` → `tools/cad/out/boat<N>/mast_*.step|stl`
  (out/ 은 git 에 안 올림. 프린트용 STL 확정본은 `docs/전달용/마스트_stl/` 에 복사해 둔다)
- `layout.py` — 갑판 위 박스 구성 시절의 배치도 (보관용)
- `concept_recessed.py` — **현행** 구상도: 갑판 덮은 오픈 헐, 박스는 갑판 아래 → `docs/전달용/배치도/concept_deck.png`
- 선체 STL 분석(거널 폭·오픈 헐 판정)은 2026-09-15 세션에서 trimesh 로 수행. 선체 STL 자체(4MB)는 저장소에 넣지 않았다 — 선체 담당 보관.

요구사항·결정 근거: `docs/전달용/하드웨어_배치_요구사항.md`
