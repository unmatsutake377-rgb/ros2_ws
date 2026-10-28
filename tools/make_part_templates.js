// 보고서 분담 — 파트별 워드 양식 4개 (IMU·GPS·LiDAR·카메라)
// 실행: NODE_PATH=<docx 모듈 경로> node tools/make_part_templates.js            → 빈 양식  양식_<센서>.docx
//       NODE_PATH=<docx 모듈 경로> node tools/make_part_templates.js --drafts   → 초안/<센서>.md 를 양식 모양으로  초안_<센서>.docx
//       ... --drafts --strip   → [근거: …] 표시를 지운 제출용 모양(검수 끝난 뒤)
// 스타일(글꼴·제목·표)은 tools/make_report_docx.js 와 같다. 합칠 때 모양이 바뀌지 않게.
// "현재 원고" 는 00_현재원고_Ⅲ소프트웨어.md 에서 절 번호로 잘라 넣는다(손으로 옮기지 않는다 → 원고와 안 어긋남).
const fs = require('fs');
const path = require('path');
const { Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell, WidthType, ShadingType, HeadingLevel, AlignmentType, BorderStyle } = require('docx');

const ROOT = path.resolve(__dirname, '..');
const DIR = path.join(ROOT, 'docs/전달용/보고서_분담');
const SRC = fs.readFileSync(path.join(DIR, '00_현재원고_Ⅲ소프트웨어.md'), 'utf8').split('\n');

// ── 스타일: make_report_docx.js 와 동일 ──
const FONT = 'Malgun Gothic'; const TW = 9360;
const b = { style: BorderStyle.SINGLE, size: 6, color: '000000' }; const borders = { top: b, bottom: b, left: b, right: b };
const t = (text, o = {}) => new TextRun({ text, font: FONT, size: 20, ...o });
const P = (text, o = {}) => new Paragraph({ children: [t(text, o.run || {})], spacing: { after: 100, line: 300 }, ...(o.p || {}) });
const B = (text) => new Paragraph({ children: [t('•  ' + text)], spacing: { after: 60, line: 290 }, indent: { left: 360, hanging: 240 } });
const NOTE = (text) => new Paragraph({ children: [t('▶ ' + text, { color: '1F7A3A', size: 18, italics: true })], spacing: { after: 100 } });
const GUIDE = (text) => new Paragraph({ children: [t('[작성 안내] ' + text, { color: '888888', size: 18 })], spacing: { after: 80, line: 280 } });
const H1 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_1, children: [t(s, { bold: true, size: 32 })], spacing: { before: 0, after: 200 } });
const H2 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_2, children: [t(s, { bold: true, size: 26 })], spacing: { before: 240, after: 120 } });
const H3 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_3, children: [t(s, { bold: true, size: 22 })], spacing: { before: 160, after: 80 } });
const H4 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_4, children: [t(s, { bold: true, size: 20 })], spacing: { before: 140, after: 60 } });
const CAP = (s) => new Paragraph({ children: [t(s, { size: 18, color: '555555' })], alignment: AlignmentType.CENTER, spacing: { after: 160 } });
const FIGBOX = (s) => new Paragraph({ children: [t('[그림 자리] ' + s, { color: '888888' })], alignment: AlignmentType.CENTER, spacing: { before: 200, after: 200 },
  border: { top: b, bottom: b, left: b, right: b } });
const cell = (ch, w, fill) => new TableCell({ width: { size: w, type: WidthType.DXA }, borders, margins: { top: 60, bottom: 60, left: 90, right: 90 }, shading: fill ? { fill, type: ShadingType.CLEAR, color: 'auto' } : undefined, children: Array.isArray(ch) ? ch : [ch] });
const para = (text, o = {}) => new Paragraph({ children: [t(text, o.run || {})], alignment: o.align || AlignmentType.LEFT });
function table(head, rows, widths) {
  const hr = new TableRow({ tableHeader: true, children: head.map((h, i) => cell(para(h, { run: { bold: true }, align: AlignmentType.CENTER }), widths[i], 'E7E6E6')) });
  const rs = rows.map(r => new TableRow({ children: r.map((v, j) => cell(String(v).split('\n').map(l => para(l)), widths[j])) }));
  return new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: widths, rows: [hr, ...rs] });
}
const SP = () => new Paragraph({ children: [t('')], spacing: { after: 60 } });
const blanks = (n, k) => Array.from({ length: n }, () => Array(k).fill('____'));
const W = (...r) => { const s = r.reduce((a, c) => a + c, 0); return r.map(x => Math.round(x * TW / s)); };

// ── 현재 원고에서 절 하나 잘라오기 (### 2.1) ... 다음 ### 또는 ## 전까지) ──
function current(head) {
  const i = SRC.findIndex(l => l.startsWith(head));
  if (i < 0) throw new Error('현재 원고에 없음: ' + head);
  const lvl = head.match(/^#+/)[0].length;
  const out = []; let tb = null;
  const flush = () => { if (tb) { const w = W(...tb[0].map(() => 1)); out.push(table(tb[0], tb.slice(1), w), SP()); tb = null; } };
  for (let j = i + 1; j < SRC.length; j++) {
    const l = SRC[j];
    const m = l.match(/^(#+) /); if (m && m[1].length <= lvl) break;
    if (l.startsWith('|')) {
      if (/^\|(\s*-+\s*\|)+\s*$/.test(l) || /^\|---/.test(l)) continue;
      const cells = l.replace(/^\|\s*|\s*\|\s*$/g, '').split(/\s*\|\s*/);
      if (!tb) tb = [cells]; else tb.push(cells); continue;
    }
    flush();
    if (!l.trim()) continue;
    if (l.startsWith('#')) out.push(H4(l.replace(/^#+\s*/, '')));
    else if (l.startsWith('- ')) out.push(B(l.slice(2)));
    else if (/^그림 Ⅲ-/.test(l)) out.push(CAP(l));
    else out.push(P(l));
  }
  flush();
  return out;
}
const CUR = (head) => [NOTE('아래 ▶ 표시 두 줄 사이는 지금 보고서 원고다. 이걸 바탕으로 고쳐 쓴다. 틀린 곳은 근거를 달아 고친다.'), ...current(head), NOTE('— 현재 원고 끝 —')];

// ── 8항목 뼈대 ──
function eight(n, p) {
  return [
    H4(`${n}.1) 장비 사양과 선정 이유`),
    GUIDE('사양 수치는 데이터시트 문서명 또는 코드 행을 출처로 단다. 선정 이유는 2~3문장.'),
    table(['항목', 'A배 (1번배)', 'B배 (2번배)', '출처'], [['모델', ...p.model, '____'], ['인터페이스', '____', '____', '____'], ['출력 주기', '____', '____', '____'], [p.precisionLabel || '정밀도·분해능', '____', '____', '____'], ['노트북 연결', '____', '____', '____']], W(2, 3, 3, 2.4)),
    SP(), P('____ (선정 이유)'),
    H4(`${n}.2) 코드 출처`),
    GUIDE('지시서의 출처 표를 옮기고 보강한다. AI 에게 출처를 추측시키지 않는다. 외부 드라이버는 내부 구현을 설명하지 않는다.'),
    table(['패키지', '원래 출처', '작년부터', '올해 바꾼 것', '근거 커밋'], p.origin, W(2, 2.6, 1.1, 3.4, 1.4)),
    H4(`${n}.3) 데이터 흐름`),
    GUIDE('입력 토픽 → 처리 노드 → 출력 토픽. 이름은 코드 표기 그대로.'),
    table(['토픽', '입력/출력', '메시지 타입', '주기', 'QoS', '상대 노드'], blanks(5, 6), W(2.4, 1.2, 2, 1, 1.4, 2)),
    FIGBOX(p.fig1), CAP(`그림 Ⅲ-__ ${p.fig1}`),
    H4(`${n}.4) 처리 알고리즘`),
    GUIDE(p.algo),
    P('____'),
    H4(`${n}.5) 주요 파라미터`),
    GUIDE('값은 yaml 기준. 위치는 파일:행. 근거는 실측·시뮬·설계 이유 중 무엇인지.'),
    table(['이름', '값', '위치 (파일:행)', '정한 근거'], blanks(6, 4), W(2.4, 1.2, 2.8, 3)),
    H4(`${n}.6) 이상 상황 대응`),
    GUIDE('센서가 끊기거나 값이 이상할 때 코드가 하는 일. "모르면 발행하지 않는다" 원칙과 연결.'),
    table(['상황', '감지 방법', '코드의 동작', '근거'], blanks(4, 4), W(2, 2.4, 3.4, 1.8)),
    H4(`${n}.7) 2025 대비 개선점`),
    GUIDE('9bff661(작년 최종본) 이후 커밋 목록과 diff 에서만 뽑는다. 추측 금지.'),
    table(['작년 (2025)', '문제', '올해 (2026)', '근거 커밋'], blanks(4, 4), W(2.6, 2.6, 2.8, 1.4)),
    H4(`${n}.8) 검증과 시험 결과`),
    GUIDE('단위 테스트는 개수와 검증 내용. 실측은 아직 없으면 ____ 그대로 둔다.'),
    table(['테스트 파일', '개수', '검증 내용'], p.tests.map(x => [x, '____', '____']), W(4, 1, 5)),
    SP(),
    table(['실측 항목', 'A배', 'B배', '측정일'], p.measure.map(x => [x, '____', '____', '____']), W(4.4, 1.6, 1.6, 1.4)),
  ];
}
const appendix = () => [
  H2('부록 (제출본에서 팀장이 지운다)'),
  H3('A. A배·B배 차이'), table(['항목', 'A배 (1번배)', 'B배 (2번배)', '근거'], blanks(3, 4), W(2.4, 2.6, 2.6, 1.8)),
  H3('B. 확인 필요 목록'), GUIDE('빈칸으로 둔 값, 근거를 못 찾은 주장을 전부 적는다.'), table(['항목', '왜 비었나', '언제 채우나'], blanks(4, 3), W(3, 3.6, 2.4)),
  H3('C. 다른 담당과 겹치는 내용'), GUIDE('지시서 경계표에서 남의 담당으로 정한 내용을 썼다면 여기 적는다.'), table(['내용', '겹치는 담당', '처리'], blanks(3, 3), W(4, 2, 3)),
];
function cover(name, sheet, sections) {
  return [
    new Paragraph({ children: [t('KABOAT 2026 설계보고서', { bold: true, size: 28 })], alignment: AlignmentType.CENTER, spacing: { after: 80 } }),
    new Paragraph({ children: [t(`Ⅲ장 소프트웨어 — 분담 양식: ${name}`, { bold: true, size: 36 })], alignment: AlignmentType.CENTER, spacing: { after: 200 } }),
    table(['담당자', '마감', '지시서', '채울 절'], [['____', '____', sheet, sections]], W(2, 2, 2.6, 3.4)),
    SP(),
    NOTE('이 파일에 바로 채워서 낸다. 파일 이름: 원고_' + name + '_<이름>.docx. 그림은 PNG 로 따로 낸다.'),
    NOTE('[작성 안내] 회색 글씨는 지우고 채운다. ____ 는 실측이 없으면 그대로 둔다. 숫자를 지어내지 않는다.'),
    NOTE('수치·동작 설명 끝에 [근거: 파일경로:행] 을 단다. 합칠 때 대조하고 제출본에서 지운다.'),
    NOTE('글꼴·제목·표 모양은 본 보고서와 같다. 스타일을 바꾸지 않는다. 절 번호와 제목도 바꾸지 않는다.'),
  ];
}
const sensorRow = (rows) => [H2('1) 시스템 구성'), H3('1.3) 센서 — 담당 줄만'),
  GUIDE('장비 사진을 직접 찍어 넣고, 용도 문장을 보강한다. 다른 줄은 다른 담당이 쓴다.'),
  table(['장비 사진', '장비명', '용도'], rows.map(r => ['(사진)', r[0], r[1]]), W(2.2, 3, 4.16))];

// ── 파트 정의 ──
const PARTS = {
  IMU: {
    sheet: '01_IMU.md', sections: 'Ⅲ-1.3 IMU 줄, Ⅲ-2.1',
    body: () => [
      ...sensorRow([['iAHRS RB-SDA-v1 9축', '요(yaw). yaw_mux 가 /imu/yaw 단독 발행']]),
      H2('2) 센서 처리'), H3('2.1) 좌표계·헤딩'), ...CUR('### 2.1)'),
      ...eight('2.1', {
        model: ['iAHRS RB-SDA-v1', 'iAHRS RB-SDA-v1'], precisionLabel: '요 정밀도·드리프트',
        origin: [['iahrs_driver', '외부. ⚠️ 원출처 담당자 확인 (package.xml 관리자 wbjin)', '있음', '/imu/yaw → /imu/yaw_raw, 부팅 0점화·GPS 덮어쓰기 파라미터화(기본 OFF), /imu/mag_heading 추가, NAV-PVT 토픽명 수정', 'ace1e33, b433662, ed2a144'],
          ['ship_goal_angle', '작년 우리 코드', '있음', '시작값 None, stale 시 발행 중단, 주기 파라미터화', '____'],
          ['ssf_heading', '올해 새로 만듦', '없음', '전부 신규 (yaw_mux, heading_logic)', 'ace1e33 외']],
        fig1: '/imu/yaw_raw → yaw_mux → /imu/yaw → ship_goal_angle → /yaw_error. 작년 구조와 나란히',
        algo: 'heading_source 선택지별 계산, 각도 규약(0=진북, 시계방향), 단위벡터 평균, 자기편각 적용 조건, COG 오프셋 추정 조건.',
        tests: ['src/ssf_heading/test/test_heading_logic.py'],
        measure: ['mount_offset_deg (조립 완료 후 정북 정렬)', 'invert_yaw 확정', '재부팅 전후 /imu/yaw_raw 일치 여부', '대회장 자기편각 (°)', '정지·직진·선회 중 헤딩 흔들림 (°)'],
      }),
    ],
  },
  GPS: {
    sheet: '02_GPS.md', sections: 'Ⅲ-1.3 GPS 줄, Ⅲ-2.2 (지오펜스 생성 포함)',
    body: () => [
      ...sensorRow([['A배 u-blox C94-M8P + ANN-MB / B배 u-blox ZED-F9P + ANN-MB', 'NTRIP RTK 위치·COG']]),
      H2('2) 센서 처리'), H3('2.2) GPS RTK'), ...CUR('### 2.2)'),
      NOTE('현재 원고 2.3 의 지오펜스 문장 (생성 부분만 이 절로 옮긴다):'), B('지오펜스는 경계를 가짜 LiDAR 로 바꿔 ranges = min(real, geofence) 로 병합. 전방 ±60° 에서 2.0 m 안이면 STOP_HOLD.'),
      ...eight('2.2', {
        model: ['u-blox C94-M8P + ANN-MB', 'u-blox ZED-F9P + ANN-MB'], precisionLabel: 'RTK 정밀도·갱신율',
        origin: [['ublox', '외부. u-blox ROS 드라이버 (package.xml 저자 Johannes Meyer, 관리자 Gareth Cross)', '있음', 'B배 F9P 설정·배별 인자, F9P 해상 모델, 배별 설정 전달·EEPROM 저장 제거', 'c1b0d81, ed2a144, c7c0c59'],
          ['ntrip_client', '외부. LORD-MicroStrain ntrip_client', '있음', '실행 경로 수정, 계정을 저장소 밖 ~/.ssf/ 로 (diff 보지 말 것)', '89818a5, f98a8f9'],
          ['north_goal_angle', '작년 우리 코드', '있음', 'fix 검증, LOS·필터, 지오펜스, 웨이포인트 파일화', '02944d8, 9221750 외']],
        fig1: 'ntrip_client → 수신기 → /ublox_gps_node/fix → gps_guard → gps_filter → LOS → /north_goal_angle_tp · /candidate_angle · /wp_mode',
        algo: '아래 네 소절로 나눠 쓴다: ① 수신기와 RTK 보정 ② fix 검증(Null Island 사고)과 필터 3단계 ③ 웨이포인트 파일·녹화와 LOS·ILOS 유도 ④ 지오펜스 광선 생성(원뿔 방식을 버린 이유). 소절마다 그림이나 표 하나.',
        tests: ['test_gps_guard.py', 'test_gps_filter.py', 'test_los_logic.py', 'test_waypoint_loader.py', 'test_waypoint_recorder.py', 'test_mission_gate.py'].map(f => 'src/north_goal_angle/test/' + f),
        measure: ['첫 fix 까지 시간 (s)', '위성 수', 'RTK FLOAT 위치 산포 (m)', 'RTK FIXED 위치 산포 (m)', '웨이포인트 도착 오차 (m)'],
      }),
      FIGBOX('지오펜스: 경기장 모서리에서 원뿔 방식은 정면이 뚫리고, 광선 방식은 막힌다'), CAP('그림 Ⅲ-__ 지오펜스 광선 방식'),
    ],
  },
  LiDAR: {
    sheet: '03_LiDAR.md', sections: 'Ⅲ-1.3 LiDAR 줄, Ⅲ-2.3, 4.4, 5(센서 감시), 6.1',
    body: () => [
      ...sensorRow([['RPLIDAR A3 — A배 개체 80BF / B배 개체 918A', '장애물·부표 거리, 갭 탐색, 25 m. 각분해능이 개체마다 달라 셀 기반 튜닝값은 배별']]),
      H2('2) 센서 처리'), H3('2.3) LiDAR 처리'), ...CUR('### 2.3)'),
      GUIDE('2.3 은 "신호 처리"(스캔 받기, inf·nan, median, 지오펜스 병합)만. 경로 선택은 4.4 에 쓴다.'),
      ...eight('2.3', {
        model: ['RPLIDAR A3 (개체 80BF)', 'RPLIDAR A3 (개체 918A)'], precisionLabel: '회전 주기·각분해능',
        origin: [['rplidar_ros', '외부. Slamtec 공식 RPLIDAR ROS 드라이버', '있음', 'A3 실행 파일 정리만. 드라이버 코드 그대로 (19cfde7 기준으로 볼 것)', '673c20a'],
          ['ship_direction', '작년 우리 코드', '있음', '페일세이프 분리, 시간 투표 OFF, median, 지오펜스 병합, 후진·rate limit, half_width 0.45→0.29', '02bfc39, 849c570, b678154, 000e46c 외']],
        fig1: '/scan → 정면 정렬 → inf·nan 치환 → (감속용) median / (회피용) 팽창 → 지오펜스 min 병합',
        algo: '스캔 정면(LIDAR_FORWARD_DEG), inf·nan 처리, median 필터를 감속 신호에만 거는 이유와 시뮬 수치, 지오펜스 병합과 경계 정지.',
        tests: ['src/ship_direction/test/test_avoidance_filters.py', 'src/ship_direction/test/test_failsafe_logic.py'],
        measure: ['부표 탐지 최대거리 (m)', '물보라 기각 수 /obstacle_reject_count (회/분)', '1 m 안팎 게이트 통과 성공률'],
      }),
      H2('4) 임무별 알고리즘'), H3('4.4) 장애물 회피 — ship_direction'), ...CUR('### 4.4)'),
      GUIDE('갭 선택 전체: 안전 구간 → 호 길이와 r_edge → min_required_width → 목표 방위 최근접 선택 → 특수 명령(5000·20000·50000) → 후진·쿨다운·rate limit. 표는 지금 것을 코드와 대조해 고친다.'),
      FIGBOX('갭 선택 개념도: 스캔 → 장애물 팽창 → 안전 구간 → 폭 판정 → 목표에 가까운 구간'), CAP('그림 Ⅲ-__ 갭 선택'),
      H2('5) 페일세이프'), ...CUR('## 5)'),
      GUIDE('이 중 센서 감시(SensorWatch: 경고 0.7 s·정지 3.0 s·연속 N·복구, 독립 타이머)와 시간 투표·TTC 폐기 이유만 보강한다. 워치독·비상정지는 팀장.'),
      FIGBOX('페일세이프 상태도: 정상 → 경고(감속) → 정지 → 복구'), CAP('그림 Ⅲ-__ 센서 페일세이프'),
      H2('6) 시뮬레이션 및 시험 결과'), H3('6.1) 시뮬레이션 스윕'), ...CUR('### 6.1)'),
      GUIDE('tools/sim/README.md 와 숫자를 대조한다. 조건(시드·폭·clearance·탐지거리·게이트폭)과 고른 값의 이유.'),
    ],
  },
  카메라: {
    sheet: '04_카메라.md', sections: 'Ⅲ-1.3 카메라 두 줄, Ⅲ-2.4, 4.5(표식 판정)',
    body: () => [
      ...sensorRow([['Luxonis OAK-1 PoE FF (HFOV 69° 스펙)', '게이트·부표·도크 색/형상 방위각. hfov_deg 는 camera_info 로 실측 교체'], ['Intel RealSense D455', '수동 조종 화면 (예비 카메라)']]),
      H2('2) 센서 처리'), H3('2.4) 영상 처리 파이프라인'), ...CUR('### 2.4)'),
      GUIDE('7단계 표를 코드와 한 줄씩 대조해 고친다. 표는 지우지 말고 고친다.'),
      ...eight('2.4', {
        model: ['OAK-1 PoE FF (자율, 1대 배정) / D455 (임시·수동 화면)', '동일'], precisionLabel: '해상도·화각',
        origin: [['realsense-ros', '외부. Intel RealSense 공식 ROS 드라이버', '있음', '바꾸지 않음', '—'],
          ['depthai-ros (OAK)', '외부. Luxonis 공식, apt 설치 (저장소에 없음)', '없음', '아직 미설치. 런북만', '—'],
          ['color_shape_detector', '작년 우리 코드', '있음', 'depth 의존 제거, 화각 파라미터화·실측 80.32°, 상주형 게이팅, HSV 단일화, 확인 창·solidity', 'c75f3c4, a1c1cb1, 1908f7c, 086c125, 52494fb, c54b3c9 외']],
        fig1: '7단계 파이프라인 블록도: 입력 → 색 분리 → 후보 추출 → 형상 판정 → 확인 → 방위각 변환 → 발행',
        algo: '검출기 3종의 담당 모드와 출력, 상주형 모드 게이팅(작년 subprocess 방식의 문제), HSV 단일 출처, 형상 판정 기준, DetectionConfirmer(최근 M 중 N), 픽셀→각도 식과 화각 실측.',
        tests: ['test_dock_logic.py', 'test_hsv_ranges.py', 'test_mode_gate.py', 'test_vision_geom.py'].map(f => 'src/color_shape_detector/test/' + f).concat(['tools/dock_eval.py --selftest']),
        measure: ['색별 정탐·오탐 혼동행렬 (프레임)', '야외 HSV 확정값', 'OAK 실측 hfov_deg', '확인 프레임 깜빡임 벤치'],
      }),
      FIGBOX('같은 장면의 원본 / HSV 마스크 / 검출 결과 3장 (튜닝 도구 화면)'), CAP('그림 Ⅲ-__ 색 분리 예시'),
      H2('4) 임무별 알고리즘'), H3('4.5) 도킹 — ship_dock'), ...CUR('### 4.5)'),
      GUIDE('이 중 표식 판정(검출기 dock 의 색·형상 일치, 유효값 밖 ERROR, 확인 로직)만 보강한다. 접근 제어·접안 거리는 팀장.'),
    ],
  },
};

const docStyles = { default: { document: { run: { font: FONT, size: 20 } } },
  paragraphStyles: [
    { id: 'Heading1', name: 'Heading 1', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 32, bold: true, font: FONT }, paragraph: { spacing: { after: 200 }, outlineLevel: 0 } },
    { id: 'Heading2', name: 'Heading 2', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 26, bold: true, font: FONT }, paragraph: { outlineLevel: 1 } },
    { id: 'Heading3', name: 'Heading 3', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 22, bold: true, font: FONT }, paragraph: { outlineLevel: 2 } },
    { id: 'Heading4', name: 'Heading 4', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 20, bold: true, font: FONT }, paragraph: { outlineLevel: 3 } } ] };

// ── 초안 md → docx ── (`코드` 는 Consolas)────────────────────────────────────────────
//  초안 md 뼈대: "## 1.3) …", "## 2.x) …", "### 2.x.k) …", "#### …", "## 4.x) …", "## 5) …", "## 6.1) …", "## 부록 …"
//  인라인 **굵게**, [근거: …] 는 회색 작은 글씨(--strip 이면 삭제). "그림: …" 한 줄 = 그림 자리.
const STRIP = process.argv.includes('--strip');
function runs(text, base = {}) {
  const out = [];
  const re = /(\*\*[^*]+\*\*|\[근거:[^\]]*\]|`[^`]+`)/g; let last = 0; let m;
  while ((m = re.exec(text))) {
    if (m.index > last) out.push(t(text.slice(last, m.index), base));
    const tok = m[0];
    if (tok.startsWith('**')) out.push(...runs(tok.slice(2, -2), { ...base, bold: true }));
    else if (tok.startsWith('`')) out.push(new TextRun({ text: tok.slice(1, -1), font: 'Consolas', size: (base.size || 20) - 2, bold: base.bold, color: base.color }));
    else if (!STRIP) out.push(t(tok, { ...base, size: 16, color: '7F7F7F' }));
    last = m.index + tok.length;
  }
  if (last < text.length) out.push(t(text.slice(last), base));
  return out.length ? out : [t('')];
}
const RP = (text) => new Paragraph({ children: runs(text), spacing: { after: 100, line: 300 } });
const RB = (text) => new Paragraph({ children: [t('•  '), ...runs(text)], spacing: { after: 60, line: 290 }, indent: { left: 360, hanging: 240 } });
function rtable(rows) {
  const head = rows[0]; const w = W(...head.map(() => 1));
  const hr = new TableRow({ tableHeader: true, children: head.map((h, i) => cell(new Paragraph({ children: runs(h, { bold: true }), alignment: AlignmentType.CENTER }), w[i], 'E7E6E6')) });
  const rs = rows.slice(1).map(r => new TableRow({ children: head.map((_, j) => cell(new Paragraph({ children: runs(r[j] || '') }), w[j])) }));
  return new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: w, rows: [hr, ...rs] });
}
function draftBody(md) {
  const L = md.split('\n'); const out = []; let tb = null; const seen = new Set();
  const parent = (h) => { if (!seen.has(h)) { seen.add(h); out.push(H2(h)); } };
  const flush = () => { if (tb) { out.push(rtable(tb), SP()); tb = null; } };
  for (const l of L) {
    if (l.startsWith('|')) {
      if (/^\|[\s:|-]+\|\s*$/.test(l)) continue;
      const cells = l.replace(/^\|\s*|\s*\|\s*$/g, '').split(/\s*\|\s*/);
      (tb = tb || []).push(cells); continue;
    }
    flush();
    if (!l.trim() || /^# /.test(l)) continue;
    let m;
    if ((m = l.match(/^## (.*)$/))) {
      const h = m[1].trim();
      if (h.startsWith('1.3)')) { parent('1) 시스템 구성'); out.push(H3(h)); }
      else if (/^2\.\d\)/.test(h)) { parent('2) 센서 처리'); out.push(H3(h)); }
      else if (/^4\.\d\)/.test(h)) { parent('4) 임무별 알고리즘'); out.push(H3(h)); }
      else if (/^6\.\d\)/.test(h)) { parent('6) 시뮬레이션 및 시험 결과'); out.push(H3(h)); }
      else if (h.startsWith('부록')) { parent('부록 (제출본에서 팀장이 지운다)'); out.push(H3(h.replace(/^부록\s*/, ''))); }
      else out.push(H2(h));
    } else if ((m = l.match(/^### (.*)$/))) out.push(H4(m[1]));
    else if ((m = l.match(/^#### (.*)$/))) out.push(new Paragraph({ children: runs(m[1], { bold: true }), spacing: { before: 100, after: 60 } }));
    else if ((m = l.match(/^\s*[-*] (.*)$/))) out.push(RB(m[1]));
    else if ((m = l.match(/^그림:\s*(.*)$/))) out.push(FIGBOX(m[1]), CAP('그림 Ⅲ-__ ' + m[1]));
    else out.push(RP(l.trim()));
  }
  flush();
  return out;
}
function draftCover(name, sheet, sections) {
  return [
    new Paragraph({ children: [t('KABOAT 2026 설계보고서', { bold: true, size: 28 })], alignment: AlignmentType.CENTER, spacing: { after: 80 } }),
    new Paragraph({ children: [t(`Ⅲ장 소프트웨어 — ${name} 초안${STRIP ? '' : ' (검토용)'}`, { bold: true, size: 36 })], alignment: AlignmentType.CENTER, spacing: { after: 200 } }),
    table(['검토자', '검토일', '지시서', '절'], [['____', '____', sheet, sections]], W(2, 2, 2.6, 3.4)),
    SP(),
    NOTE('코드·문서·git 기록만으로 쓴 초안이다. ____ 는 실측이 있어야 채울 수 있는 자리다.'),
    NOTE('회색 [근거: 파일:행] 은 검토용이다. 그 줄을 열어 내용이 맞는지 확인한다. 제출본에서는 지운다.'),
    NOTE('검토자가 할 일: ① 근거 대조 ② 장비 사진 ③ 그림 ④ 실측 빈칸 ⑤ 부록 B 확인 필요 해결.'),
  ];
}

(async () => {
  if (process.argv.includes('--drafts')) {
    for (const [name, p] of Object.entries(PARTS)) {
      const src = path.join(DIR, '초안', `${name}.md`);
      if (!fs.existsSync(src)) { console.log('없음(건너뜀):', path.relative(ROOT, src)); continue; }
      const children = [...draftCover(name, p.sheet, p.sections), H1('Ⅲ. 소프트웨어'), ...draftBody(fs.readFileSync(src, 'utf8'))];
      const doc = new Document({ styles: docStyles, sections: [{ properties: { page: { size: { width: 11906, height: 16838 }, margin: { top: 1440, bottom: 1440, left: 1273, right: 1273 } } }, children }] });
      const out = path.join(DIR, STRIP ? `제출용_${name}.docx` : `초안_${name}.docx`);
      fs.writeFileSync(out, await Packer.toBuffer(doc));
      console.log('wrote', path.relative(ROOT, out));
    }
    return;
  }
  for (const [name, p] of Object.entries(PARTS)) {
    const children = [...cover(name, p.sheet, p.sections), H1('Ⅲ. 소프트웨어'), ...p.body(), ...appendix()];
    const doc = new Document({ styles: docStyles, sections: [{ properties: { page: { size: { width: 11906, height: 16838 }, margin: { top: 1440, bottom: 1440, left: 1273, right: 1273 } } }, children }] });
    const out = path.join(DIR, `양식_${name}.docx`);
    fs.writeFileSync(out, await Packer.toBuffer(doc));
    console.log('wrote', path.relative(ROOT, out));
  }
})();
