# 시험 프린트 쿠폰 (v4 기준, 2026-09-16)

절차·합격기준: `docs/절차/마스트_시험프린트.md`
생성: `python3 tools/cad/mast.py` → `python3 tools/cad/fit_test.py`

| 파일 | 크기(mm) | 우선순위 |
|---|---|---|
| `fit_cradle_d455.stl` | 133 × 37 × 33.5 | 1 — 실물 D455 로 바로 확인 |
| `fit_pocket_imu.stl` | 82 × 68 × 20 | 2 — 실물 iAHRS 로 확인 |
| `fit_pocket_lid.stl` | 65.6 × 51.6 × 3 | 2 (뚜껑) |
| `fit_joint_plug.stl` | 60 × 60 × 40 | 3 — 이음 여유 0.4 확인 |
| `fit_joint_socket.stl` | 40 × 40 × 42 | 3 (짝) |
| `fit_cradle_oak.stl` | 90.9 × 42 × 86 | 4 — OAK 미발주, 목업으로 간접 |
| `fit_oak_mockup.stl` | 81.9 × 31 × 116.9 | 4 (OAK 외형 목업, 속 빔) |

⚠️ 쿠폰만 고치지 말 것. 결과가 안 맞으면 `tools/cad/mast.py` 파라미터를 고치고 다시 생성한다.
