// 실행: NODE_PATH=<docx 모듈 경로> node tools/make_report_docx.js docs/전달용/KABOAT2026_보고서_양식_SSF.docx
// 2026 KABOAT 보고서 양식 — 2025 S.S.F A/B 보고서 구조(Ⅰ~Ⅶ) 그대로. 현재 확정값 미리 채움, 빈칸은 ____.
const fs = require('fs');
const { Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell, WidthType, ShadingType, HeadingLevel, AlignmentType, BorderStyle, PageBreak } = require('docx');
const FONT = 'Malgun Gothic'; const TW = 9360; // A4 세로, 여백 2.54cm 기준 본문폭
const b = { style: BorderStyle.SINGLE, size: 6, color: '000000' }; const borders = { top: b, bottom: b, left: b, right: b };
const t = (text, o = {}) => new TextRun({ text, font: FONT, size: 20, ...o });
const P = (text, o = {}) => new Paragraph({ children: [t(text, o.run || {})], spacing: { after: 100, line: 300 }, ...(o.p || {}) });
const NOTE = (text) => new Paragraph({ children: [t('▶ ' + text, { color: '1F7A3A', size: 18, italics: true })], spacing: { after: 100 } });
const BLANK = (text) => new Paragraph({ children: [t(text, { color: '888888' })], spacing: { after: 100 } });
const H1 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new PageBreak(), t(s, { bold: true, size: 32 })], spacing: { before: 0, after: 200 } });
const H2 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_2, children: [t(s, { bold: true, size: 26 })], spacing: { before: 240, after: 120 } });
const H3 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_3, children: [t(s, { bold: true, size: 22 })], spacing: { before: 160, after: 80 } });
const cell = (ch, w, fill) => new TableCell({ width: { size: w, type: WidthType.DXA }, borders, margins: { top: 60, bottom: 60, left: 90, right: 90 }, shading: fill ? { fill, type: ShadingType.CLEAR, color: 'auto' } : undefined, children: Array.isArray(ch) ? ch : [ch] });
const para = (text, o = {}) => new Paragraph({ children: [t(text, o.run || {})], alignment: o.align || AlignmentType.LEFT });
function table(head, rows, widths, opt = {}) {
  const hr = new TableRow({ tableHeader: true, children: head.map((h, i) => cell(para(h, { run: { bold: true }, align: AlignmentType.CENTER }), widths[i], 'E7E6E6')) });
  const rs = rows.map(r => new TableRow({ children: r.map((v, j) => cell(String(v).split('\n').map(l => para(l, { align: opt.center && opt.center.includes(j) ? AlignmentType.CENTER : AlignmentType.LEFT })), widths[j])) }));
  return new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: widths, rows: [hr, ...rs] });
}
const eq = (rows) => table(['장비 사진', '장비명', '용도'], rows.map(r => ['(사진)', r[0], r[1]]), [2200, 3000, 4160]);

const children = [
  // 표지
  new Paragraph({ children: [t('자율운항보트경진대회 2026', { size: 28 })], alignment: AlignmentType.CENTER, spacing: { before: 2400, after: 200 } }),
  new Paragraph({ children: [t('KABOAT 2026', { size: 48, bold: true })], alignment: AlignmentType.CENTER, spacing: { after: 600 } }),
  new Paragraph({ children: [t('홍익대학교 조선해양공학과', { size: 26 })], alignment: AlignmentType.CENTER, spacing: { after: 200 } }),
  new Paragraph({ children: [t('TEAM _ S.S.F ____ (A / B)', { size: 30, bold: true })], alignment: AlignmentType.CENTER, spacing: { after: 1200 } }),
  new Paragraph({ children: [t('조장 : ____', { size: 22 })], alignment: AlignmentType.CENTER }),
  new Paragraph({ children: [t('조원 : ____', { size: 22 })], alignment: AlignmentType.CENTER }),
  new Paragraph({ children: [t('지도교수 : 김승남', { size: 22 })], alignment: AlignmentType.CENTER }),
  // 목차
  H1('목차'),
  ...['Ⅰ. 서론', 'Ⅱ. 선형 설계 및 제작', '   1) 주요 치수 설정  1.1) 조건 및 고려 사항  1.2) 주요 설정 치수', '   2) 선형 설계  2.1) 설계 도면  2.2) 배수용적·복원성·캐비테이션 평가', '   3) 제작 과정', '   4) 장비 및 구성', 'Ⅲ. 소프트웨어', '   1) 종합 임무 및 알고리즘  1.1) 센서  1.2) 노드', '   2) 소프트웨어 장비 및 구성', 'Ⅳ. 회로', '   1) 회로도 구상 및 비상 정지 장치 구성  1.1) 회로도 구성  1.2) 비상 정지 장치 구성', '   2) 회로 장비 및 구성', 'Ⅴ. 최종 소모 금액', 'Ⅶ. 최종 결과'].map(s => P(s)),
  NOTE('2025 S.S.F A·B 보고서(docs/전달용/양식/작년보고서_2025_SSF_A.pdf) 목차 그대로. 번호 Ⅵ 이 비어 있는 것도 작년과 같다. 필요하면 Ⅵ 시험·수조 결과로 채운다.'),

  H1('Ⅰ. 서론'),
  BLANK('(작년 서론 문단을 2026 기준으로 고쳐 쓴다 — 올해 변경점: 배 2척 동시 운용, OAK 비전 카메라 도입, 접촉기 기반 비상정지, ROS2 Humble 노드 재구성)'),

  H1('Ⅱ. 선형 설계 및 제작'),
  H2('1) 주요 치수 설정'), H3('1.1) 조건 및 고려 사항'),
  table(['요구 크기 및 무게 조건', '값'], [['길이 [m]', 'L ≥ 1.0 (작년 규정. 2026 배 크기 규정 변경분 ____ — 과제 S5)'], ['폭 [m]', '요구사항 없음'], ['높이 [m]', '요구사항 없음 (2026 변경분 ____)'], ['무게 [kg]', '요구사항 없음']], [4000, 5360]),
  H3('1.2) 주요 설정 치수'),
  table(['최종 주요 제원', '1번배 (S.S.F A)', '2번배 (S.S.F B)'], [['LOA [m]', '1.66 (09-28 줄자)', '1.77 (10-24 줄자)'], ['BOA [m]', '0.57 (스러스터·러더 포함)', '0.535'], ['깊이 D [m]', '0.38 (선체 17 + 스러스터 11 + 러더 10)', '0.14 (선체)'], ['추진기', 'T200 ×2 (선미 ±220)', 'T200 ×4 (선미 ±195, 선수 x450 ±135)'], ['전비 질량 [kg]', '____', '____'], ['흘수 / 건현 [mm]', '____ / ____', '____ / ____ (35kg 추정 74 / 66)']], [2600, 3380, 3380]),
  NOTE('출처: CLAUDE_상세 §8, 스러스터_배치_해석 부록(10-24). 질량·흘수는 D1 실측 후.'),
  H2('2) 선형 설계'), H3('2.1) 설계 도면'), BLANK('(3D 모델링 측면도·정면도·평면도 — STL 보유자 삽입)'),
  H3('2.2) 배수용적·복원성·캐비테이션 평가'), BLANK('(GM = KB + BM − KG. 작년 식 그대로. 스러스터 h/D 환기 판정: 흘수 74mm 에 축 50mm 아래 → h/D 1.63 — 스러스터_배치_해석)'),
  H2('3) 제작 과정'), BLANK('(FRP 적층 → 퍼티 → 갑판 → 관통부 실란트 — 사진)'),
  H2('4) 장비 및 구성'),
  eq([['PLA+ 필라멘트 (마스트 PETG/ASA)', '마스트·크레들 프린트'], ['FRP 춉매트 / 얀크로스', '선체 적층'], ['방수 실리콘', '관통부·결합부 마감'], ['방수 컨트롤 박스 400×300×200', '전장품 수납']]),

  H1('Ⅲ. 소프트웨어'),
  H2('1) 종합 임무 및 알고리즘'), BLANK('(노드 구조 그림 — CLAUDE.md §3 토픽 표 기준: north_goal_angle → ship_gate/turn/dock/back → ship_direction → motor_control → ssf_bridge)'),
  H3('1.1) 센서'),
  eq([['RPLIDAR A3 (80BF=1번배 / 918A=2번배)', '장애물·부표 거리, 갭 탐색'], ['iAHRS RB-SDA-v1', '요(yaw) — yaw_mux 가 /imu/yaw 단독 발행'], ['u-blox ZED-F9P + ANN-MB (RTK)', '위치·속도. NTRIP FIXED ±1.5cm'], ['Luxonis OAK-1 PoE FF 69°', '부표 색·도크 표식 인식 (비전)'], ['Intel RealSense D455', '수동 조종 화면']]),
  H3('1.2) 노드'), BLANK('(미션별 노드 설명 — 작년 서술 틀 유지: 가) 항로추종 ship_gate 나) 위치유지 ship_back 다) 선회 ship_turn 라) 도킹 ship_dock 마) 회피 ship_direction. half_width 0.29 / clearance 0.25 시뮬 근거)'),
  H2('2) 소프트웨어 장비 및 구성'),
  eq([['Lenovo IdeaPad Slim 3 15IAH8 ×2', 'Main Computer — 배당 1대, ROS2 Humble'], ['Arduino Mega 2560 R3 ×2', 'RC 수신·ESC 신호·LED·비상정지 감지 (펌웨어 ssf_boat.ino)'], ['FlySky FS-i6 / FS-iA6B', '수동 조종 · 모드 전환(RC 전용)'], ['ipTIME UH308 유전원 허브', 'LiDAR·IMU·GPS·D455 USB 확장'], ['NETmate NM-UCL02 (AX88179)', 'OAK 이더넷 ↔ 노트북 USB-C']]),

  H1('Ⅳ. 회로'),
  H2('1) 회로도 구상 및 비상 정지 장치 구성'), H3('1.1) 회로도 구성'),
  BLANK('(그림: docs/전달용/배치도/회로도_통합.png 2026-10-26 판 삽입 — ① 주 전력 ② 차단 ③ 제어·카메라·허브 ④ 결선표)'),
  H3('1.2) 비상 정지 장치 구성'),
  BLANK('(LS MD-30a 접촉기 2극 — 배터리 A/B ⊕ 한 가닥씩. 코일 24V 는 SZH-BPM001 승압. 무선 릴레이 SRD-12VDC-SL-C + 버튼 NC 직렬. 보조접점 21/22 → Mega 38. 카메라·허브는 접촉기 앞 분기라 비상정지 후에도 살아 있음)'),
  H2('2) 회로 장비 및 구성'),
  eq([['T200 Thruster ×6 (1번배 2 / 2번배 4)', '추진'], ['BlueRobotics BESC30-R3 ×8', '양방향 ESC, 공장 설정'], ['DXF 4S 14.8V 8400mAh 120C ×5', '배당 2 + 예비 1'], ['LS MD-30a 접촉기 DC 24V', '비상정지 주 전력 차단'], ['SZH-BPM001 승압 9~20V→24V 3A', '접촉기 코일 전원'], ['RF 무선 릴레이 + 비상정지 버튼', '무선·유선 비상정지'], ['REVOTECH PSE5502G PoE 인젝터', 'OAK 급전 (DC 12-48V → 48V 802.3af)'], ['SZWENGAO WG-1224S0505 5V 5A', '허브 급전 (5V 고정, IP68)'], ['BF303 퓨즈홀더 + 5×20 2A/3A', '인젝터·허브 입력 보호'], ['WS2812 LED 스트립', '룰 표시등 (초록/노랑/빨강)']]),

  H1('Ⅴ. 최종 소모 금액'),
  table(['', '품목', '수량', '가격', '합계'], [
    ['1', 'OAK-1 PoE FF', '2', '661,000', '1,322,000'], ['2', 'PoE 인젝터 PSE5502G', '2', '61,000', '122,000'], ['3', 'NETmate NM-UCL02', '2', '18,490', '36,980'],
    ['4', 'USB 3.0 C↔A 케이블 (벨킨)', '3', '5,000~10,000', '15,000~30,000'], ['5', '승압 컨버터 SZH-BPM001', '2', '13,200', '26,400'], ['6', 'Arduino Mega 2560 R3', '1', '64,900', '64,900'],
    ['7', 'RF 무선스위치', '1', '27,200', '27,200'], ['8', 'EC5 피그테일', '3', '3,000', '9,000'], ['9', 'ipTIME UH308', '1', '27,900', '27,900'],
    ['10', 'SZWENGAO 5V 모듈 + DC잭 세트', '1+1', '10,120 + 4,500', '14,620'], ['11', 'DC 5.5×2.1 수 플러그', '3', '990', '2,970'], ['12', 'BF303 ×2 + 퓨즈 2A/3A ×20', '2+20', '1,300 / 110', '4,800'],
    ['13', 'RPLIDAR A3 (중고, 10-07)', '1', '170,000', '170,000'], ['', '총 구매 금액', '', '', '1,843,770 ~ 1,858,770 (지원금 1,414,100 / 자부담 나머지)'],
  ], [500, 3800, 900, 1900, 2260], { center: [0, 2] }),
  NOTE('출처: docs/전달용/구매_목록.md (10-26). 작년 보유분(T200·ESC·배터리·접촉기·릴레이·GPS·IMU·D455·노트북)은 작년 보고서 Ⅴ 에 있음 — 올해 신규 구매만. 선체 재료비(FRP·퍼티·페인트)는 선체 담당이 추가.'),

  H1('Ⅶ. 최종 결과'),
  BLANK('(전체 설계 모습 측면도·정면도·평면도 + 수조·야외 시험 결과 + 대회 결과)'),
  NOTE('세팅 당일(부품 전부 도착 후 1일)에 채우는 절: Ⅱ-4 장비, Ⅲ-2 장비, Ⅳ-1.1 회로도, Ⅳ-2 장비, Ⅴ 비용. 수조·야외 후: Ⅱ-1.2 질량·흘수, Ⅲ-1.2 파라미터 실측값, Ⅶ.'),
];
const doc = new Document({
  styles: { default: { document: { run: { font: FONT, size: 20 } } },
    paragraphStyles: [
      { id: 'Heading1', name: 'Heading 1', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 32, bold: true, font: FONT }, paragraph: { spacing: { after: 200 }, outlineLevel: 0 } },
      { id: 'Heading2', name: 'Heading 2', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 26, bold: true, font: FONT }, paragraph: { outlineLevel: 1 } },
      { id: 'Heading3', name: 'Heading 3', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 22, bold: true, font: FONT }, paragraph: { outlineLevel: 2 } } ] },
  sections: [{ properties: { page: { size: { width: 11906, height: 16838 }, margin: { top: 1440, bottom: 1440, left: 1273, right: 1273 } } }, children }],
});
Packer.toBuffer(doc).then(buf => { fs.writeFileSync(process.argv[2], buf); console.log('wrote', process.argv[2], buf.length); });
