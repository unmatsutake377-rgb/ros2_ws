// 발표자료(PPT 4:3, 12장) 생성기 — 보고서 docx 와 같은 수치·그림만 쓴다.
// 실행: NODE_PATH=<pptxgenjs 모듈 경로> PPTX_SKILL_APPLY_THEME=<apply_theme.js 경로> node tools/make_slides_pptx.js <출력.pptx> <A|B>
//   (apply_theme.js 는 pptx 스킬 scripts/ 에 있는 테마 색 주입기. 없으면 Office 기본 팔레트로 나온다)
// 빈 칸(수조 시험 뒤 기입)은 노란 상자 "[ … ]" 로 표시한다. 채우면 PH() 호출을 값으로 바꾼다.
// 구성은 docs/전달용/보고서_분담/발표자료_구성표.md 표 그대로.
'use strict';
const fs = require('fs');
const path = require('path');
const pptxgen = require('pptxgenjs');

const [, , OUT, BOAT] = process.argv;
if (!OUT || !['A', 'B'].includes(BOAT)) {
  console.error('사용법: node tools/make_slides_pptx.js <출력.pptx> <A|B>');
  process.exit(1);
}
const isA = BOAT === 'A';
const ROOT = path.resolve(__dirname, '..');
const FIG = (p) => path.join(ROOT, 'docs/전달용/배치도', p);
const TEAMFIG = (p) => path.join(ROOT, 'docs/전달용/보고서_분담/그림_팀원', p);
const HULL = (p) => path.join(ROOT, 'docs/전달용/선도_1번배', p);
const exists = (p) => { try { return fs.statSync(p).isFile(); } catch { return false; } };

// ── 배별 값 (보고서 docx 와 동일) ───────────────────────────────────────────────
const B = isA ? {
  team: 'S.S.F A', no: '1번배', leader: '도건윤',
  members: '김은우, 김정한, 김하은, 박주현, 황준연, 석현, 정우성',
  loa: '1.66', boa: '0.55', depth: '0.255 m (선미, 추진기 포함)',
  thr: 'T200 ×2 · 선미 좌우 대칭 (중심선 ±120 mm, 간격 240 mm)',
  mass: '30 (추정)', draft: '선미 45 · 중앙 95 · 선수 105 (부상 시험)', freeboard: '선미 105 · 중앙 125 · 선수 135',
  kb: '0.0706', bm: '0.2197', km: '0.2903', wl: '102.6 mm (STL 계산)', cb: '0.485',
  thrustTotal: '94 N (2기)', S: '0.637 m²', lwl: '1.510 m',
  layer: 'FRP 적층', hullImg: HULL('1번배_선형_3D.png'), planImg: FIG('hull_plan_A.png'),
  thrImg: FIG('thruster_asbuilt_A.png'), resImg: FIG('resistance_A.png'),
  circImg: TEAMFIG('회로_구성도_A.png'), estopImg: TEAMFIG('비상정지_구성도_A.png'),
  draftMeasured: true,
  cost: [
    ['RPLIDAR A3', '1', '800,000'], ['T200 스러스터', '2', '670,634'], ['OAK-1 PoE FF', '1', '661,000'],
    ['GPS ZED-F9P + 안테나', '1+1', '610,900'], ['IMU iAHRS', '1', '198,000'], ['배터리 4S LiPo', '2', '155,200'],
    ['조종기 FS-i6', '1', '129,000'], ['ESC BESC30-R3', '2', '106,248'], ['그 외 (Mega·박스·인젝터·컨버터 등)', '—', '≈ 270,000'],
  ],
  costTotal: '3,603,967 ~ 3,608,967 원 + 선체 재료',
} : {
  team: 'S.S.F B', no: '2번배', leader: '박유진',
  members: '박윤서, 차준혁, 이송언, 이태훈, 김현지, 전홍윤, 황예진',
  loa: '1.77', boa: '0.522', depth: '0.19 m (선체)',
  thr: 'T200 ×4 · V자형 (선미 간격 280, 선수 측 125, 전후 365 mm)',
  mass: '35 (추정)', draft: null, freeboard: null,
  kb: '0.037', bm: '0.20', km: '0.24', wl: '68.2 mm (STL 계산)', cb: null,
  thrustTotal: '188 N (4기)', S: '0.724 m²', lwl: '1.552 m',
  layer: '카본섬유 적층', hullImg: FIG('hull_plan_B.png'), planImg: FIG('hull_plan_B.png'),
  thrImg: FIG('thruster_asbuilt_B.png'), resImg: FIG('resistance_B.png'),
  circImg: TEAMFIG('회로_구성도_B.png'), estopImg: TEAMFIG('비상정지_구성도_B.png'),
  draftMeasured: false,
  cost: [
    ['T200 스러스터', '4', '1,341,268'], ['OAK-1 PoE FF', '1', '661,000'], ['RPLIDAR A3 (중고)', '1', '300,000'],
    ['ESC BESC30-R3', '4', '212,496'], ['배터리 4S LiPo', '2', '155,200'], ['Arduino Mega 2560 R3', '1', '64,900'],
    ['PoE 인젝터', '1', '61,000'], ['무선 릴레이 GIE-WT3R', '1', '23,760'], ['노트북·GPS·IMU', '—', '학과 보유'],
  ],
  costTotal: '2,862,539 ~ 2,867,539 원 + 선체 재료',
};

// ── 테마 ────────────────────────────────────────────────────────────────────────
// 틀은 작년(SSF KABOAT 2025) PPT 그대로: 흰 배경, 상단 가는 남색 선, 좌상단 남색 제목 20pt + 짧은 남색 바,
// 우상단 홍익대 로고, 하단 남색 띠, 우하단 쪽번호. 크기만 경기규정 3.1(5) "표준크기 4:3" 으로.
const NAVY = '052E65', NAVY2 = '213A62';
const THEME = {
  name: 'SSF 2026',
  headFontFace: '맑은 고딕', bodyFontFace: '맑은 고딕',
  colors: {
    dk1: '000000', lt1: 'FFFFFF', dk2: NAVY, lt2: 'F2F2F2',
    accent1: NAVY2, accent2: 'C0392B', accent3: '2E7D32', accent4: '7A4B00', accent5: 'FFF5C2', accent6: '666666',
    hlink: NAVY, folHlink: '666666',
  },
};
const pres = new pptxgen();
pres.layout = 'LAYOUT_4x3';            // 10 × 7.5 in
pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
pres.title = `KABOAT 2026 ${B.team} 발표자료`;
pres.author = `홍익대학교 ${B.team}`;
const C = pres.SchemeColor;
const W = 10, H = 7.5, M = 0.5;
const LOGO = TEAMFIG('로고_홍익대.png');   // 작년 PPT 에서 가져온 홍익대 로고 (418×150)

// 작년 틀 요소 (위치는 16:9 → 4:3 로 가로만 맞춤)
const FRAME = [
  { rect: { x: 0.26, y: 0.25, w: 9.48, h: 0.03, fill: { color: C.accent1 }, line: { color: C.accent1 } } },        // 상단 선
  { rect: { x: 0.18, y: 0.56, w: 0.35, h: 0.08, fill: { color: C.text2 }, line: { color: C.text2 } } },           // 제목 밑 짧은 바
  { rect: { x: 0, y: 7.32, w: W, h: 0.18, fill: { color: C.accent1 }, line: { color: C.accent1 } } },             // 하단 띠
  { image: { x: 7.6, y: 0.29, w: 2.11, h: 0.76, path: LOGO } },
];
pres.defineSlideMaster({
  title: 'CONTENT', background: { color: THEME.colors.lt1 },
  objects: [
    ...FRAME,
    { placeholder: { options: { name: 'title', type: 'title', x: 0.4, y: 0.39, w: 7.0, h: 0.44, fontSize: 20, bold: true, color: C.text2, margin: 0, valign: 'middle' }, text: '' } },
  ],
  slideNumber: { x: 9.0, y: 6.9, w: 0.6, h: 0.35, fontSize: 12, color: C.text1, align: 'right' },
});
pres.defineSlideMaster({
  title: 'TITLE', background: { color: THEME.colors.lt1 },
  objects: [
    { rect: { x: 0.26, y: 0.49, w: 9.48, h: 0.03, fill: { color: C.accent1 }, line: { color: C.accent1 } } },
    { rect: { x: 0, y: 7.04, w: W, h: 0.46, fill: { color: C.accent1 }, line: { color: C.accent1 } } },
  ],
  slideNumber: { x: 9.0, y: 6.6, w: 0.6, h: 0.35, fontSize: 12, color: C.text1, align: 'right' },
});

// ── 도우미 ─────────────────────────────────────────────────────────────────────
let sn = 0;
function slide(title, section) {
  const s = pres.addSlide({ masterName: 'CONTENT', sectionTitle: section });
  sn += 1;
  s.addText(title, { placeholder: 'title' });
  return s;
}
// 빈 칸: 수조 시험 뒤 채운다 (연노랑 상자)
function PH(s, x, y, w, h, label, opt = {}) {
  s.addText(`( ${label} )`, {
    x, y, w, h, fill: { color: C.accent5 }, color: C.accent4, fontSize: opt.fontSize || 11,
    align: 'center', valign: 'middle', isTextBox: true, margin: 0.04, objectName: `빈칸 ${label}`,
  });
}
function bullets(s, items, x, y, w, h, opt = {}) {
  const arr = items.map((t, i) => ({
    text: t, options: { bullet: { indent: 14 }, breakLine: i < items.length - 1, paraSpaceAfter: opt.gap ?? 6 },
  }));
  s.addText(arr, { x, y, w, h, fontSize: opt.fontSize || 13, color: C.text1, valign: 'top', isTextBox: true, margin: 0, objectName: opt.name || '본문' });
}
function sub(s, text, x, y, w) {
  s.addText(text, { x, y, w, h: 0.32, fontSize: 13, bold: true, color: C.text2, isTextBox: true, margin: 0, objectName: `소제목 ${text}` });
}
function img(s, file, x, y, w, h, caption) {
  if (!exists(file)) { PH(s, x, y, w, h, `그림 없음: ${path.basename(file)}`); return; }
  const sz = pngSize(file);
  let iw = w, ih = h;
  if (sz) { const r = sz.w / sz.h; if (w / h > r) { iw = h * r; } else { ih = w / r; } }
  const ix = x + (w - iw) / 2, iy = y + (h - ih) / 2;
  s.addImage({ path: file, x: ix, y: iy, w: iw, h: ih, objectName: caption || path.basename(file) });
  if (caption) s.addText(caption, { x, y: y + h + 0.03, w, h: 0.28, fontSize: 9.5, color: C.accent6, align: 'center', isTextBox: true, margin: 0, objectName: '캡션' });
}
function pngSize(file) {
  try {
    const b = fs.readFileSync(file);
    if (b.toString('ascii', 1, 4) === 'PNG') return { w: b.readUInt32BE(16), h: b.readUInt32BE(20) };
    let i = 2;
    while (i < b.length) {
      if (b[i] !== 0xff) { i++; continue; }
      const m = b[i + 1];
      if (m >= 0xc0 && m <= 0xcf && m !== 0xc4 && m !== 0xc8 && m !== 0xcc) return { h: b.readUInt16BE(i + 5), w: b.readUInt16BE(i + 7) };
      i += 2 + b.readUInt16BE(i + 2);
    }
  } catch { /* 무시 */ }
  return null;
}
function table(s, rows, x, y, w, colW, opt = {}) {
  const fs_ = opt.fontSize || 11.5;
  const data = rows.map((r, ri) => r.map((c) => {
    const ph = typeof c === 'object' && c && c.ph;
    const txt = ph ? `( ${c.ph} )` : String(c);
    const o = { fontSize: fs_, color: C.text1, valign: 'middle', margin: [3, 5, 3, 5] };
    if (ri === 0 && opt.header !== false) { o.bold = true; o.fill = { color: C.background2 }; }
    else if (ph) { o.color = C.accent4; o.fill = { color: C.accent5 }; o.align = 'center'; }
    return { text: txt, options: o };
  }));
  s.addTable(data, { x, y, w, colW, border: { type: 'solid', pt: 0.5, color: 'BFBFBF' }, rowH: opt.rowH || 0.32, objectName: opt.name || '표' });
}
// 소제목 + 불릿 묶음 (카드 대신)
function card(s, x, y, w, h, head, body, opt = {}) {
  sub(s, head, x, y, w);
  if (Array.isArray(body)) bullets(s, body, x, y + 0.38, w, opt.bodyH || (h - 0.4), { fontSize: opt.fontSize || 11.5, gap: 3, name: `카드 본문 ${head}` });
  else s.addText(body, { x, y: y + 0.38, w, h: h - 0.4, fontSize: opt.fontSize || 11.5, color: C.text1, valign: 'top', isTextBox: true, margin: 0, objectName: `카드 본문 ${head}` });
}

// ── 1. 표지 (작년 구도: 왼쪽 사진 2장 + 남색 삼각형, 오른쪽 제목) ─────────────
pres.addSection({ title: '소개' });
{
  const s = pres.addSlide({ masterName: 'TITLE', sectionTitle: '소개' });
  s.addShape(pres.ShapeType.triangle, { x: -0.6, y: 1.68, w: 2.37, h: 1.45, rotate: 90, fill: { color: C.accent1 }, line: { color: C.accent1 }, objectName: '왼쪽 삼각형' });
  s.addShape(pres.ShapeType.triangle, { x: 0.93, y: 4.55, w: 3.65, h: 2.49, fill: { color: C.accent1 }, line: { color: C.accent1 }, objectName: '아래 삼각형' });
  // 표지 사진 2장 = 작년 PPT 표지 그대로 (작년 파일에서 잘라낸 이미지, 팀장 11-02 요청)
  img(s, TEAMFIG('표지_작년_1.jpg'), 1.0, 0.6, 4.3, 2.9);
  img(s, TEAMFIG('표지_작년_2.jpg'), 0.0, 2.9, 3.33, 4.0);
  s.addText('2026 자율운항보트경진대회', { x: 4.6, y: 3.6, w: 5.1, h: 0.6, fontSize: 26, bold: true, color: C.text2, isTextBox: true, margin: 0, objectName: '대회명' });
  s.addText('S.S.F (Symbio Ship to Future)', { x: 4.6, y: 4.25, w: 5.1, h: 0.45, fontSize: 16, color: C.text1, isTextBox: true, margin: 0, objectName: '팀명' });
  s.addText(`홍익대학교 조선해양공학과 · TEAM ${B.team} (${B.no})`, { x: 4.6, y: 4.8, w: 5.1, h: 0.4, fontSize: 13, bold: true, color: C.text1, isTextBox: true, margin: 0 });
  s.addText([
    { text: `조장 ${B.leader}`, options: { breakLine: true } },
    { text: `조원 ${B.members}`, options: { breakLine: true } },
    { text: '지도교수 권순범', options: {} },
  ], { x: 4.6, y: 5.3, w: 5.1, h: 1.2, fontSize: 11.5, color: C.accent6, valign: 'top', isTextBox: true, margin: 0, paraSpaceAfter: 3, objectName: '팀 명단' });
}

// ── 2. 설계철학·임무 우선순위 (Ⅱ-1) ────────────────────────────────────────────
pres.addSection({ title: '선형 설계 및 제작' });
{
  const s = slide('설계 요구조건과 설계 반영', '선형 설계 및 제작');
  bullets(s, [
    '두 척 모두 단동선 — 지난 대회 쌍동선·V형 단동선 비교 끝에 선종 통일, 설계·제작 경험 공유',
    isA ? '추진기 4기에서 2기로: 단순화·경량화, 선미 차동추력으로 선회' : '추진기 4기 V자형: 선수 측 안쪽·선미 측 바깥, 좌·우 2기씩 같은 신호',
    '임무 우선순위는 코스 순서대로: 게이트, 위치유지, 탐색, 장애물 회피, 도킹, 토너먼트',
  ], M, 1.3, 9, 1.3, { fontSize: 13 });
  table(s, [
    ['규정 (경기규정 4.2 보트 검사)', '본 설계'],
    ['L ≥ 1.0 m, H ≤ 1.0 m', `LOA ${B.loa} m × BOA ${B.boa} m, 깊이 ${B.depth}`],
    ['100~110 % 부력 흘수선 표시', B.draftMeasured ? '부상 시험 흘수 45·95·105 mm → 110 % 수선 109.1 mm 계산' : { ph: '2번배 흘수 실측 후 기입' }],
    ['복원성 확보, 중량물 고정', '배터리·방수박스 선체 내부 고정, 좌우 중량 균형'],
    ['DC 모터 + 축전지, ≤ 24 V DC', '4S LiPo 14.8 V (만충 16.8 V), T200'],
  ], M, 2.75, 9, [3.3, 5.7], { fontSize: 11 });
  table(s, [
    ['경기장 구성물', '치수', '설계 반영'],
    ['부표 높이 / 폭', '1.0 m / 0.5 m', 'LiDAR 장착 높이 = 부표 높이, 카메라 시야 확보'],
    ['자율운항 장애물 간격', '약 1.0 m', '통과 최소 폭 0.29×2 + 0.25 = 0.83 m'],
    ['도크 간격', '1.0 m', `선폭 ${B.boa} m → 좌우 여유 약 ${(((1.0 - Number(B.boa)) / 2)).toFixed(2)} m`],
    ['도킹 표식', '3형상 × 5색 중 당일 지정', 'vision.yaml 에 6색 HSV·3형상 판정 보유'],
  ], M, 4.75, 9, [2.2, 2.3, 4.5], { fontSize: 11 });
}

// ── 3. 선종·주요 제원·복원성 (Ⅱ-2, 3) ────────────────────────────────────────
{
  const s = slide('주요 제원 및 초기 복원성', '선형 설계 및 제작');
  table(s, [
    ['항목', '값'],
    ['LOA / BOA [m]', `${B.loa} / ${B.boa}`],
    ['깊이 D', B.depth],
    ['추진기', B.thr],
    ['전비 질량 [kg]', B.mass],
    ['흘수 [mm]', B.draft || { ph: '수조 실측 (선수·중앙·선미)' }],
    ['건현 [mm]', B.freeboard || { ph: '수조 실측' }],
  ], M, 1.3, 4.9, [1.5, 3.4], { fontSize: 10.5, rowH: 0.36 });
  img(s, B.planImg, M, 4.05, 4.9, 2.4, `그림 ${B.no} 선형`);
  // 복원성
  const x2 = 5.7, w2 = 3.8;
  s.addText('초기 복원성 (STL 정수력 계산)', { x: x2, y: 1.3, w: w2, h: 0.35, fontSize: 13, bold: true, color: C.text2, isTextBox: true, margin: 0 });
  table(s, [
    ['KB / BM [m]', `${B.kb} / ${B.bm}`],
    ['KM = KB + BM [m]', B.km],
    ['수선 높이', B.wl],
    ['GM = KM − KG [m]', { ph: '경사 시험' }],
    ['횡요 고유주기 [s]', { ph: '롤 감쇠 시험' }],
  ], x2, 1.7, w2, [1.7, 2.1], { fontSize: 10.5, header: false, rowH: 0.36 });
  card(s, x2, 3.75, w2, 2.7, '경사 시험 (수조, 담수)', [
    '추 w 를 중심선에서 d 만큼 좌·우로 옮겨 기울기 θ 측정',
    'GM = w·d / (W·tan θ)',
    '마스트 조립체 무게·높이로 KG 보정',
  ], { fontSize: 11, bodyH: 1.3 });
  PH(s, x2 + 0.15, 5.7, w2 - 0.3, 0.6, 'w · d · θ · GM 값 + 경사 사진', { fontSize: 11 });
}

// ── 4. 추진기 배치·저항·침몰방지 (Ⅱ-3.3, 3.4, 4) ────────────────────────────
{
  const s = slide('추진기 배치 · 저항 추정 · 침몰방지', '선형 설계 및 제작');
  img(s, B.thrImg, M, 1.3, 4.4, 1.9, `그림 Ⅱ-2 ${B.no} 추진기 배치 (실제 장착)`);
  img(s, B.resImg, 5.1, 1.3, 4.4, 2.2, '그림 Ⅱ-3 속도별 점성 저항과 가용 추력');
  bullets(s, [
    `T200 최대 추력 1기 약 47 N (14.8 V 보간) → 가용 추력 ${B.thrustTotal}`,
    'Fn 0.4 (약 1.5 m/s) 부터 조파 저항 급증 — 순항 속도는 그 아래',
  ], M, 3.85, 4.4, 1.25, { fontSize: 10.5, gap: 3, name: '추력 불릿' });
  bullets(s, [
    `침수 표면적 ${B.S}, 수선 길이 ${B.lwl}. ITTC-1957 × 형상계수 1.2 — 2.0 m/s 에서도 점성 저항은 추력의 몇 %`,
    isA ? '차동추력 선회: 좌·우 출력 차로 선회 모멘트, 선미 간격 240 mm' : '선미 2기는 중심선에서 멀리(선회 모멘트), 선수 2기는 선형 따라 안쪽',
    '잠김 깊이: 흘수·파랑 고려, 환기(ventilation) 없는 높이',
  ], 5.1, 3.85, 4.4, 1.25, { fontSize: 10, gap: 2 });
  card(s, M, 5.15, 9, 1.7, '침몰방지·수밀', [
    '전장품 전부 400×300×200 방수박스 (뚜껑 = 해치, 개스킷·물끊기). 갑판 관통은 마스트 구멍 하나로 모아 실란트 마감',
    '선체·스러스터 관통부, 갑판 부착부, 박스 관통부 방수 마감 후 수조에서 수밀 확인',
    isA ? 'FRP 적층 선체 + 방수제(노루페인트) 외벽, 관통부 실리콘' : '카본섬유 적층 선체 + 방수제 외벽, 관통부 실리콘(TYTAN FIX2)',
  ], { fontSize: 11 });
  s.addNotes('선체 모델에서 배수량에 맞는 수선을 찾아 침수 표면적을 계산하고, ITTC-1957 마찰 저항식에 형상계수 1.2 를 곱해 속도별 점성 저항을 냈다. T200 추력은 제조사 사양을 14.8 V 로 환산해 1기당 약 47 N. 2.0 m/s 에서도 점성 저항은 추력의 몇 % 이므로 속도를 제한하는 것은 조파 저항이며, Fn 0.4(약 1.5 m/s) 아래에서 순항 속도를 정한다. 예상 질문: 조파 저항은 왜 계산 안 했나 → 소형 선체는 경험식이 부정확해 급증 경계만 표시하고 실제 최고 속도는 물 위 시험으로 확인. 형상계수 1.2 근거 → 소형 단동선 통상 범위 1.1~1.3 의 가운데.');
}

// ── 5. 제작·조립 과정 (Ⅱ-5) ─────────────────────────────────────────────────
{
  const s = slide('제작 과정', '선형 설계 및 제작');
  const steps = [['① 몰드 제작', `공정_${BOAT}_몰드.jpg`, 'PLA+ 3D 프린팅 몰드, Six10 에폭시 접합'],
    [`② ${B.layer}`, `공정_${BOAT}_적층.jpg`, isA ? 'FRP 춉매트 / 얀크로스 적층·보강' : '카본섬유 적층 — 경량화·강성'],
    ['③ 수밀', `공정_${BOAT}_수밀.jpg`, '방수제 도포, 관통부 실리콘 씰링'],
    ['④ 부착물', `공정_${BOAT}_부착물.jpg`, '나사 고정 + 에폭시·실리콘 혼합 마감']];
  steps.forEach(([h, f, d], i) => {
    const x = M + i * 2.3;
    img(s, TEAMFIG(f), x, 1.3, 2.1, 2.9);
    s.addText(h, { x, y: 4.3, w: 2.1, h: 0.35, fontSize: 13, bold: true, color: C.text2, isTextBox: true, margin: 0 });
    s.addText(d, { x, y: 4.65, w: 2.1, h: 0.8, fontSize: 11, color: C.text1, valign: 'top', isTextBox: true, margin: 0 });
  });
  card(s, M, 5.6, 9, 1.2, '재료', isA
    ? 'PLA+ 필라멘트(몰드) · FRP 춉매트/얀크로스 · 방수 실리콘·실란트 · 방수제(노루페인트) · Six10 에폭시 · 방수 컨트롤박스 400×300×200'
    : 'PLA+ 필라멘트(몰드) · 카본섬유 · 방수 실리콘(TYTAN FIX2) · 방수제(노루페인트) · Six10 에폭시 · 방수 컨트롤박스 400×300×200', { fontSize: 11.5 });
}

// ── 6. 시스템 구성·센서 (Ⅲ-1, 7) ────────────────────────────────────────────
pres.addSection({ title: '소프트웨어' });
{
  const s = slide('시스템 구성 및 센서', '소프트웨어');
  img(s, FIG('node_graph.png'), M, 1.3, 5.2, 3.5, '그림 Ⅲ-1 ROS2 노드·토픽 구조');
  table(s, [
    ['센서 / 장치', '역할', '주기'],
    ['RPLIDAR A3', '거리 — 갭 선택 회피, 접안 거리', '≈ 11 Hz'],
    ['iAHRS IMU', '헤딩 (자기북 + 편각)', '100 Hz'],
    ['ZED-F9P RTK GPS', '위치·웨이포인트 (NTRIP)', '1 Hz'],
    ['OAK-1 PoE', '자율 비전 — 색·형상 → 방위각', '30 fps'],
    ['RealSense D455', '수동 조종 화면 (H.264 송출)', '30 fps'],
    ['Arduino Mega', 'RC 수신·ESC PWM·LED·비상정지 감지', '—'],
  ], 5.9, 1.3, 3.6, [1.15, 1.85, 0.6], { fontSize: 9.5, rowH: 0.42 });
  bullets(s, [
    'Lenovo IdeaPad (Ubuntu 22.04, ROS2 Humble) 한 대가 스택 전체. ssf_bridge 가 USB 시리얼로 좌·우 PWM 을 Mega 로',
    '모드 전환은 조종기 스위치뿐 (WAIT / MANUAL / AUTO). 노트북엔 모드를 바꾸는 경로를 두지 않음',
    '명령 500 ms 무수신이면 펌웨어가 자동 중립. 추력 부호 반전은 ssf_bridge 한 곳',
    `실행: ros2 launch launch_files launch_files.launch.py boat:=${BOAT}`,
  ], M, 5.15, 9, 1.7, { fontSize: 11.5 });
}

// ── 7. 임무 상태기계 (Ⅲ-3) ──────────────────────────────────────────────────
{
  const s = slide('임무 상태기계', '소프트웨어');
  img(s, FIG('mission_flowchart.png'), M, 1.3, 4.6, 3.1, '그림 Ⅲ-5 종합임무 상태기계');
  table(s, [
    ['wp_mode', '임무', '노드 · 검출기', 'dwell / 시간초과'],
    ['0, 1', '게이트 (항로추종)', 'ship_gate · 빨강/초록', '3 s / 120 s'],
    ['2', '위치유지', 'ship_back · 흰색', '3 s / 120 s'],
    ['3', '탐색 (부표 선회)', 'ship_turn · 빨강', '3 s / 90 s'],
    ['5', '장애물 회피', 'ship_direction (LiDAR)', '50 s / 120 s'],
    ['7', '도킹', 'ship_dock · 지정 색·형상', '60 s / 150 s'],
    ['8', '토너먼트 자율 회피', 'ship_direction', '60 s / —'],
  ], 5.3, 1.3, 4.2, [0.7, 1.25, 1.35, 0.9], { fontSize: 9.5, rowH: 0.4 });
  bullets(s, [
    '웨이포인트 도착(반경 안 dwell 초) 또는 시간초과면 north_goal_angle 이 다음 임무로. 순서·dwell 은 waypoints.yaml 한 파일',
    '검출기는 담당 모드에서만 발행, /wp_mode 2 s 끊기면 발행 정지',
    '특수 코드: 5000 우선회 · 6000 좌선회 · 20000 미션 없음(LiDAR 회피) · 50000 정지',
    '현장 스위치: 탐색이 "정지"로 확정되면 search_behavior:=hold 로 ship_back 이 모드 3 담당',
  ], M, 4.75, 9, 2.1, { fontSize: 11.5 });
}

// ── 8. 영상 파이프라인·회피·제어기 (Ⅲ-2.4, 4) ──────────────────────────────
{
  const s = slide('영상 처리 · 장애물 회피 · 제어기', '소프트웨어');
  img(s, FIG('vision_pipeline.png'), M, 1.3, 4.4, 2.65, '그림 Ⅲ-2 영상 처리 단계 (실제 OAK 화면)');
  img(s, FIG('sim_avoid.png'), 5.1, 1.3, 4.4, 2.65, '그림 Ⅲ-7 장애물 회피 — 갭 선택');
  card(s, M, 4.3, 2.9, 2.5, '비전 (규칙 기반)', [
    'HSV 색 마스크 후 중앙값 필터',
    '윤곽 다각형 근사: 꼭짓점 수·원형도',
    '화면 위치를 방위각으로, 실측 화각 63.9°',
    '640×360 MJPEG, 검출기 26~29.5 fps',
  ], { fontSize: 10.5 });
  card(s, 3.55, 4.3, 2.9, 2.5, '회피 (ship_direction)', [
    '탐지거리 3.0 m (게이트 구간 2.0 m)',
    '장애물 양옆을 half_width + clearance 만큼 팽창',
    '폭 ≥ 0.83 m 틈 중 목표 방위에 가장 가까운 각',
    '틈 없으면 후진 후 재탐색',
  ], { fontSize: 10.5 });
  card(s, 6.6, 4.3, 2.9, 2.5, '제어기 (각도 기반 차동)', [
    '/desired_angle 오차를 좌·우 PWM 차동으로',
    '전방 거리로 감속, 한 주기 조향 변화 상한',
    'ROS <1500 = 전진, 반전은 bridge 한 곳',
    '명령 워치독 0.5 s, 끊기면 중립',
  ], { fontSize: 10.5 });
  s.addNotes('비전: 실제 OAK 640×360 화면에 검출기 코드를 그대로 적용. 빨강 범위로 마스크, 중앙값 필터로 잡티 제거, 윤곽을 다각형 근사해 꼭짓점 수와 원형도로 형상 판정, 채택한 표식의 화면 위치를 실측 화각 63.9° 로 방위각으로 변환. 거리는 LiDAR. 회피: 탐지거리 안의 장애물을 배 폭만큼 넓혀 막고, 폭 판정(0.83 m)을 통과한 틈 중 목표 방위에 가장 가까운 쪽을 고른다. 목표 방위가 이미 틈 안이면 그대로 간다. 예상 질문: 왜 YOLO 안 쓰나 → 표식이 색·단순 도형이라 규칙 기반으로 충분, 학습 데이터 불필요, 판정 근거 설명 가능. 햇빛 변화 → HSV 범위는 설정 파일 한 곳, 현장에서 수정.');
}

// ── 9. 페일세이프·시뮬 스윕 (Ⅲ-5, 6.1) ─────────────────────────────────────
{
  const s = slide('페일세이프 및 시뮬레이션', '소프트웨어');
  img(s, FIG('failsafe_timeline.png'), M, 1.3, 4.4, 1.6, '그림 Ⅲ-8 페일세이프 단계 전환 (벤치 실측)');
  img(s, FIG('sim_sweep.png'), 5.1, 1.3, 4.4, 1.6, '그림 Ⅲ-9 회피 파라미터 스윕');
  table(s, [
    ['단계', '조건 (연속 3회)', '동작'],
    ['0 정상', '—', '계산한 각도 100 %'],
    ['1 경고', '> 0.7 s 무신호', '각도 유지, 속도 0.7 배'],
    ['2 정지', '> 3.0 s 또는 이상 스캔', 'STOP_HOLD'],
  ], M, 3.3, 4.4, [0.9, 1.7, 1.8], { fontSize: 10.5, rowH: 0.34 });
  bullets(s, [
    'LiDAR 끊김: 0.70 s 경고, 2.99 s 정지, 복귀 즉시 정상. 14.3분 기록 오탐 0',
    'IMU 끊김: /yaw_error 0.8 s 뒤 멈춤 → 1.8 s 경고, 4.1 s 정지',
    'GPS fix 2.5 s 없으면 방위·지오펜스·도착 판정 정지. 비전 유실 2 s 뒤 무효값',
    '진단(/health_ok)은 로그만 — 거짓 정지 금지',
  ], M, 4.85, 4.4, 2.0, { fontSize: 10.5 });
  sub(s, '회피 파라미터 스윕 (2,400 런)', 5.1, 3.3, 4.4);
  bullets(s, [
    '선폭 5 × clearance 6 × 탐지거리 2 × 게이트폭 2, 칸마다 80 런. 채택 half_width 0.29 m · clearance 0.25 m',
    'clearance 0.20 → 접촉 1.95, 0.25 → 1.32, 0.30 → 1.46 (정체 2 %), 0.35 → 정체 28 %',
    '정체 0 % 범위에서 접촉 최소인 0.25 채택. 통과 최소 폭 0.83 m',
    '전 코스(70×20 m) 궤적 시뮬로 출발~도착 확인 (그림 Ⅲ-10)',
  ], 5.1, 3.75, 4.4, 2.6, { fontSize: 10.5 });
  s.addNotes('페일세이프: 센서가 끊기면 바로 멈추지 않고 경고(0.7 s, 속도 0.7배·조향 유지) → 정지(3.0 s) 두 단계. 오판 방지로 연속 3회 확인. 벤치 실측 LiDAR 0.70/2.99 s, IMU 1.8/4.1 s. 왜 경고 때 직진 안 시키나 → 회피 중 직진 강제는 피하던 부표에 부딪힌다. 비상정지와 다름: 이건 소프트웨어 판단, 비상정지는 접촉기가 물리적으로 추진 전원을 끊는다. 스윕: half_width·clearance 조합마다 80번 주행, 정체 0 % 범위에서 접촉 최소 조합(0.29·0.25) 채택.');
}

// ── 10. 회로도·비상정지 (Ⅳ-1, 2) ────────────────────────────────────────────
pres.addSection({ title: '회로' });
{
  const s = slide('회로 구성 및 비상정지', '회로');
  img(s, B.circImg, M, 1.3, 4.4, 2.4, `그림 Ⅳ-1 ${B.no} 전체 회로 구성도`);
  img(s, B.estopImg, 5.1, 1.3, 4.4, 2.4, '그림 Ⅳ-2 비상정지 장치 구성도');
  table(s, [
    ['계통', '경로', '비상정지 시'],
    ['추진 전력', `배터리 A·B (+) → 접촉기(2극) → ESC → T200 ×${isA ? 2 : 4}`, '차단'],
    ['비상정지 코일', '배터리 A → 24 V 승압(SZH-BPM001) → 접촉기 코일 → 무선 릴레이(NC) → 버튼(NC)', '코일 무전원 → 열림'],
    ['센서 허브 5 V', '배터리 A 접촉기 앞단 → 3 A 퓨즈 → 5 V 모듈 → USB 허브', '유지'],
    ['카메라 PoE 48 V', '배터리 A 접촉기 앞단 → 2 A 퓨즈 → PoE 인젝터 → OAK', '유지'],
  ], M, 4.1, 9, [1.4, 6.0, 1.6], { fontSize: 10.5, rowH: 0.36 });
  bullets(s, [
    '배터리 2개 (4S LiPo 14.8 V, 8400 mAh, 약 248 Wh) 가 좌·우 추진 분담. LS MD-30a 2극 접촉기: 버튼·무선이 코일 회로를 직렬로 끊어 제어기가 죽어도 동작',
    '접촉기 보조접점 → Mega 핀 38: 노트북이 차단 상태를 알고 표시등 빨강',
  ], M, 6.15, 9, 0.85, { fontSize: 10, gap: 2 });
}

// ── 11. 시험 결과 수치 (Ⅱ-3.2, Ⅲ-6, Ⅳ-2) ───────────────────────────────────
pres.addSection({ title: '시험·비용' });
{
  const s = slide('시험 결과', '시험·비용');
  s.addText('벤치·야외 (완료)', { x: M, y: 1.25, w: 4.6, h: 0.3, fontSize: 12, bold: true, color: C.text2, isTextBox: true, margin: 0 });
  table(s, [
    ['시험', '결과'],
    ['헤딩 체인', 'yaw_raw 100 Hz · yaw 50 Hz · yaw_error 20 Hz'],
    ['LiDAR 페일세이프', '0.70 s 경고 · 2.99 s 정지 · 오탐 0 / 14.3 분'],
    ['GPS RTK', 'FIXED ±0.015 m (위성 18, pDOP 1.63)'],
    ['색 검출 60 s', '빨강 1220/1220 · 초록 520/520, 5° 튐 0'],
    ['10분 연속', '센서 5종 유지, 끊김 0, CPU 여유 83 %'],
    ['수동 영상 송출', '424×240 · 30 fps · 1 Mbps, LTE 지연 113 ms'],
    ['배터리 A 분리', '센서 4종 유지, 접촉기만 열려 추진 정지'],
    isA ? ['부상 시험 흘수', '선미 45 · 중앙 95 · 선수 105 mm'] : ['부상 시험 흘수', { ph: '수조 실측' }],
  ], M, 1.6, 4.6, [1.4, 3.2], { fontSize: 9.5, rowH: 0.38 });
  s.addText('수조 시험 (6 × 6 m, 담수)', { x: 5.4, y: 1.25, w: 4.1, h: 0.3, fontSize: 12, bold: true, color: C.text2, isTextBox: true, margin: 0 });
  table(s, [
    ['항목', '값'],
    ['GM (경사 시험)', { ph: '값 [m]' }],
    ['횡요 고유주기', { ph: '값 [s]' }],
    ['조향 부호 · 후진 방향', { ph: '정상 / 반대' }],
    ['직진 속도 PWM 1400 / 1360 / 1300', { ph: 'm/s ×3' }],
    ['정지 거리 (타력)', { ph: '값 [m]' }],
    ['제자리 선회 1바퀴 시간', { ph: '우 / 좌 [s]' }],
    ['최소 선회 지름 (조향 160 / 0)', { ph: '값 [m]' }],
    ['직진 쏠림 (8.5 m 끝)', { ph: '값 [m]' }],
  ], 5.4, 1.6, 4.1, [2.3, 1.8], { fontSize: 9.5, rowH: 0.38 });
  card(s, M, 5.6, 9, 1.25, '대회 전 물 위에서 확인할 것', '조향 부호·추력 PWM 확정 · IMU 오프셋·자기편각 · LTE 핫스팟 켠 상태 RTK FIXED · 도크 접안 거리(contact_dist_m) · 무선 비상정지 도달 거리', { fontSize: 11 });
}

// ── 12. 비용 (Ⅴ) ────────────────────────────────────────────────────────────
{
  const s = slide('최종 소모 금액', '시험·비용');
  table(s, [['품목', '수량', '합계 [원]'], ...B.cost], M, 1.3, 5.6, [3.4, 0.8, 1.4], { fontSize: 11, rowH: 0.38 });
  sub(s, '총 구매 금액', 6.5, 1.3, 3.0);
  s.addText(B.costTotal, { x: 6.5, y: 1.7, w: 3.0, h: 0.9, fontSize: 15, bold: true, color: C.text1, isTextBox: true, margin: 0, valign: 'top', objectName: '총액' });
  PH(s, 6.5, 2.7, 3.0, 0.5, '선체 재료비 — 선체 담당 기입', { fontSize: 11 });
  bullets(s, [
    isA ? '센서 4종(LiDAR·GPS·IMU·OAK) 이 전체의 약 63 %' : '추진기 4기 + ESC 가 전체의 약 54 %, 노트북·GPS·IMU 는 학과 보유',
    '카메라 계통(OAK + 인젝터 + 어댑터) 약 74 만원',
    '출처: 구매 목록 + 기보유 장비 목록, 이 배에 단 장비만',
  ], 6.5, 3.5, 3.0, 2.5, { fontSize: 10.5 });
}

(async () => {
  await pres.writeFile({ fileName: OUT });
  const { applyTheme } = require(process.env.PPTX_SKILL_APPLY_THEME || path.join(__dirname, 'apply_theme.js'));
  await applyTheme(OUT, THEME);
  console.log(`${OUT} — ${sn + 1} slides`);
})().catch((e) => { console.error(e); process.exit(1); });
