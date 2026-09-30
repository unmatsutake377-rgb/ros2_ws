const fs = require('fs');
const {
  Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell, WidthType, ShadingType,
  HeadingLevel, AlignmentType, PageOrientation, ExternalHyperlink, BorderStyle, LevelFormat,
} = require('docx');

const FONT = 'Malgun Gothic';
const TW = 14570; // A4 가로, 여백 2cm
const COLS = [3000, 4000, 1100, 2300, 700, 3470];
const HEAD = ['품목', '용도 — 어디에 쓰나', '수량', '금액(원)', '우선', '구매 링크'];
const border = { style: BorderStyle.SINGLE, size: 4, color: 'A6A6A6' };
const borders = { top: border, bottom: border, left: border, right: border };

const t = (text, o = {}) => new TextRun({ text, font: FONT, size: 19, ...o });
const link = (label, url) => new ExternalHyperlink({ link: url,
  children: [new TextRun({ text: label, font: FONT, size: 19, style: 'Hyperlink', color: '0563C1', underline: {} })] });

function cell(children, w, opts = {}) {
  return new TableCell({
    width: { size: w, type: WidthType.DXA }, borders,
    margins: { top: 60, bottom: 60, left: 100, right: 100 },
    shading: opts.fill ? { fill: opts.fill, type: ShadingType.CLEAR, color: 'auto' } : undefined,
    children: Array.isArray(children) ? children : [children],
  });
}
const para = (runs, o = {}) => new Paragraph({ children: Array.isArray(runs) ? runs : [runs], ...o });

function headerRow(cols, heads, fill = '1F3864') {
  return new TableRow({ tableHeader: true, children: heads.map((h, i) =>
    cell(para(t(h, { bold: true, color: 'FFFFFF' })), cols[i], { fill })) });
}

// links: [[label,url],...]
function itemRow(r) {
  const linkRuns = [];
  r.links.forEach(([l, u], i) => { if (i) linkRuns.push(t(' · ')); linkRuns.push(link(l, u)); });
  return new TableRow({ children: [
    cell(para(t(r.name, { bold: true })), COLS[0]),
    cell(para(t(r.use)), COLS[1]),
    cell(para(t(r.qty), { alignment: AlignmentType.CENTER }), COLS[2]),
    cell(para(t(r.price), { alignment: AlignmentType.RIGHT }), COLS[3]),
    cell(para(t(r.pri, { bold: r.pri === 'P0', color: r.pri === 'P0' ? 'C00000' : '000000' }), { alignment: AlignmentType.CENTER }), COLS[4]),
    cell(para(linkRuns), COLS[5]),
  ] });
}
function itemTable(rows, subtotal) {
  const tr = [headerRow(COLS, HEAD), ...rows.map(itemRow)];
  if (subtotal) tr.push(new TableRow({ children: [
    cell(para(t('소계', { bold: true })), COLS[0], { fill: 'F2F2F2' }),
    cell(para(t('')), COLS[1], { fill: 'F2F2F2' }), cell(para(t('')), COLS[2], { fill: 'F2F2F2' }),
    cell(para(t(subtotal, { bold: true }), { alignment: AlignmentType.RIGHT }), COLS[3], { fill: 'F2F2F2' }),
    cell(para(t('')), COLS[4], { fill: 'F2F2F2' }), cell(para(t('')), COLS[5], { fill: 'F2F2F2' }),
  ] }));
  return new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: COLS, rows: tr });
}

const H1 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun({ text: s, font: FONT })] });
const H2 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_2, spacing: { before: 280, after: 100 }, children: [new TextRun({ text: s, font: FONT })] });
const P = (s, o = {}) => para(t(s, o), { spacing: { before: 80, after: 80 } });
const flow = (s) => para(new TextRun({ text: s, font: 'Consolas', size: 17, color: '404040' }),
  { spacing: { after: 100 }, shading: { fill: 'F2F2F2', type: ShadingType.CLEAR, color: 'auto' } });
const bullet = (s) => new Paragraph({ numbering: { reference: 'chk', level: 0 }, spacing: { after: 60 }, children: [t(s)] });

const DM = 'https://www.devicemart.co.kr/goods/view?no=';
const A = [
  { name: 'OAK-1 PoE FIXED-FOCUS 69°', use: '[지원금 들어옴] 자율운항 비전 카메라. 마스트 꼭대기에서 부표·표식 인식. 옵션: 32MP·US Cord 인젝터 선택 금지', qty: '2', price: '1,322,000', pri: 'P0', links: [['리얼리스토어', 'https://reallystore.net/goods/view?no=45']] },
  { name: 'PoE 인젝터 REVOTECH PSE5502G', use: '4S 배터리를 48V PoE 로 변환. 랜선 한 가닥으로 카메라 전원+데이터', qty: '2', price: '145,240', pri: 'P0', links: [['11번가', 'https://www.11st.co.kr/products/9621501833']] },
  { name: 'USB-C 기가비트 어댑터 (AX88179 칩셋)', use: '인젝터 LAN 과 노트북 연결. 노트북에 랜 포트가 없다. 1000Mbps 필수', qty: '2', price: '40,000~60,000 (개당 2~3만)', pri: 'P0', links: [['다나와 검색', 'https://search.danawa.com/dsearch.php?query=AX88179']] },
  { name: '퓨즈홀더 BF303 + 5x20 퓨즈 2A', use: '인젝터 입력 보호. 배터리 ⊕ 와 인젝터 사이', qty: '홀더 2 · 퓨즈 2', price: '2,820', pri: 'P1', links: [['홀더', DM + '14122158'], ['2A', DM + '4538']] },
];
const B = [
  { name: 'USB 3.0 C↔A 케이블 1m 이하', use: '수동 경기 조종 화면. USB 2.0 케이블이면 0fps. SS 로고·파란 단자 확인, 충전 전용 금지', qty: '2 + 예비 1', price: '15,000~30,000 (개당 5천~1만)', pri: 'P0', links: [['IN-U3TOC 1m', 'https://prod.danawa.com/info/?pcode=16803692'], ['벨킨 F2CU029bt', 'https://prod.danawa.com/info/?pcode=3404024']] },
];
const D = [
  { name: '승압 컨버터 SZH-BPM001 (입력 9~20V → 24V 3A)', use: '비상정지 접촉기 코일 구동. 4S 만충 16.8V 까지 덮는다. 확보 전까지 배터리 16V 초과 충전 금지', qty: '1 + 예비 1', price: '26,400 (개당 13,200)', pri: 'P0', links: [['디바이스마트', DM + '1330749']] },
];
const E = [
  { name: 'ipTIME UH308 전원형 USB 3.0 허브', use: '라이다·IMU·GPS USB 확장. 아두이노는 여기 꽂지 않고 노트북 직결', qty: '1', price: '27,900', pri: 'P1', links: [['다나와', 'https://prod.danawa.com/info/?pcode=2223672'], ['디바이스마트', DM + '1214178']] },
  { name: 'XL4015 강압 5V 5A + DC 플러그 5.5×2.1', use: '배터리에서 허브용 5V 생성. LM2596(3A)은 부족. 허브 연결 전 5.0~5.1V 로 조정', qty: '1', price: '3,000~7,000', pri: 'P1', links: [['메카솔루션', 'https://www.mechasolution.com/goods/goods_view.php?goodsNo=604291']] },
  { name: '퓨즈홀더 BF303 + 5x20 퓨즈 3A', use: '허브 급전(XL4015 입력) 보호 + 예비 1', qty: '홀더 2 · 퓨즈 2', price: '2,820', pri: 'P1', links: [['홀더', DM + '14122158'], ['3A', DM + '4537']] },
];

const G = [
  { name: 'Arduino Mega 2560 R3 정품 (A000067)', use: '[지원금 들어옴] 2번배 메인 제어기. RC 수신·ESC 신호·LED·비상정지 감지. 보유 1대는 1번배용', qty: '1', price: '58,000', pri: 'P0', links: [['다나와 검색', 'https://search.danawa.com/dsearch.php?query=A000067']] },
];

const SUMCOLS = [5000, 2000, 3500, 4070];
const sumRows = [
  ['A. OAK 카메라', '4', '1,510,060 ~ 1,530,060', 'P0 카메라·인젝터·어댑터, P1 퓨즈'],
  ['B. D455 조종 카메라', '1', '15,000 ~ 30,000', 'P0'],
  ['C. 라이다', '0', '0', '구매 없음 — 예비 0'],
  ['D. 비상정지', '1', '26,400', 'P0 — 만충 운용이 이것에 막혀 있다'],
  ['E. USB 허브', '3', '33,720 ~ 37,720', 'P1'],
  ['G. 제어 (Mega)', '1', '58,000', 'P0 — 2번배용. 지원금 들어옴'],
  ['합계', '9', '1,643,180 ~ 1,682,180', 'OAK 2대가 79~80%'],
];
const sumTable = new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: SUMCOLS, rows: [
  headerRow(SUMCOLS, ['계통', '품목 수', '금액(원)', '비고']),
  ...sumRows.map((r, i) => new TableRow({ children: r.map((v, j) => cell(
    para(t(v, { bold: i === sumRows.length - 1 || j === 0 }), { alignment: j === 1 ? AlignmentType.CENTER : j === 2 ? AlignmentType.RIGHT : AlignmentType.LEFT }),
    SUMCOLS[j], i === sumRows.length - 1 ? { fill: 'F2F2F2' } : {})) })),
] });

const NOCOLS = [3000, 5500, 6070];
const noRows = [
  ['C. 라이다', '현역 2대가 돈다 (09-23 팀 결정)', '예비 0. 대회 중 하나 죽으면 그 배는 못 나간다. 단선품에 "쓰지 말 것" 표시'],
  ['F. 주 전력', '08-25 재배선 끝, 09-07 물 위 시험 통과', '인젝터·허브 급전용 가는 선(22~26AWG)은 재고 확인'],
  ['G. 제어 전원', '노트북을 항상 싣는다 — Mega·수신기는 노트북 USB (Mega 본체는 위 G 에서 구매)', 'LED 밝기 40 한낮 시인성 시험. 안 보이면 UBEC 5V 3A 구매(접촉기 앞에서 분기)'],
  ['H. 마스트·기구', '필라멘트·나사 재고 또는 외부 출력', '관통부는 실란트 마감 + 드립 루프'],
  ['I. GPS', '접지판은 자체 제작', '알루미늄 Ø100~150mm, 나사 2개 고정 (RTK FIXED 조건)'],
  ['J. 배 ID', '분기에 쓰는 곳이 없다 (09-17 전수 확인)', '없음. 두 배 회로 동일'],
];
const noTable = new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: NOCOLS, rows: [
  headerRow(NOCOLS, ['계통', '왜 안 사나', '남은 일'], '595959'),
  ...noRows.map(r => new TableRow({ children: r.map((v, j) => cell(para(t(v, { bold: j === 0 })), NOCOLS[j])) })),
] });

const doc = new Document({
  styles: {
    default: { document: { run: { font: FONT, size: 20 } } },
    paragraphStyles: [
      { id: 'Heading1', name: 'Heading 1', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { size: 32, bold: true, font: FONT, color: '1F3864' }, paragraph: { spacing: { after: 120 }, outlineLevel: 0 } },
      { id: 'Heading2', name: 'Heading 2', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { size: 24, bold: true, font: FONT, color: '1F3864' }, paragraph: { outlineLevel: 1 } },
    ],
  },
  numbering: { config: [{ reference: 'chk', levels: [{ level: 0, format: LevelFormat.BULLET, text: '☐', alignment: AlignmentType.LEFT,
    style: { paragraph: { indent: { left: 400, hanging: 300 } } } }] }] },
  sections: [{
    properties: { page: { size: { width: 11906, height: 16838, orientation: PageOrientation.LANDSCAPE },
      margin: { top: 1000, bottom: 1000, left: 1134, right: 1134 } } },
    children: [
      H1('SSF 2026 구매 목록 — 계통별'),
      P('2026-09-30 기준 · 단일 출처: 저장소 docs/전달용/구매_목록.md · 가격은 09-14~09-23 다나와·공식샵 기준 추정 · 금액 = 적힌 수량 전체, VAT 포함', { color: '595959', size: 17 }),
      P('사야 할 것은 9항목, 합계 약 164~168만원이다. 아직 하나도 주문하지 않았다. 같은 계통 안에서 하나만 빠져도 그 계통은 안 돈다.', { bold: true }),
      sumTable,

      H2('A. OAK 카메라 계통 — 자율운항 비전'),
      flow('배터리 ⊕ ─ 2A 퓨즈 ─▶ PoE 인젝터 ─ 동봉 2m Cat5e ─▶ OAK-1 PoE        인젝터 LAN ─▶ AX88179 어댑터 ─▶ 노트북'),
      itemTable(A, '1,510,060 ~ 1,530,060'),
      P('랜선(Cat5e 2m)은 OAK 동봉이라 사지 않는다. 인젝터 전원은 접촉기 「앞」에서 딴다 — 비상정지는 추진만 끊는다.', { size: 18 }),

      H2('B. D455 조종 카메라 계통 — 수동 경기 화면'),
      flow('D455 ─ USB 3.0 C↔A (1m 이하) ─▶ 노트북'),
      itemTable(B, '15,000 ~ 30,000'),
      P('D455 는 1대 보유. 배마다 1대씩 쓰면 1대 추가 구매가 필요하다 — 미정.', { size: 18 }),

      H2('C. 라이다 계통 — 구매 없음'),
      P('현역 2대가 돈다(09-23 팀 결정). 예비가 0 이라 대회 중 하나 죽으면 그 배는 못 나간다. 3번째(케이블 단선)는 6핀/5핀 케이블 하나면 살릴 수 있다.', { size: 18 }),

      H2('D. 비상정지 계통'),
      flow('배터리A ⊕ (접촉기 앞) ─▶ 승압 컨버터 ─▶ 코일 A1·A2 ─▶ 무선 릴레이 COM/NC ─▶ 버튼 NC'),
      itemTable(D, '26,400'),
      P('접촉기·릴레이·버튼은 전부 보유. 08-25 주문분이 09-17 취소돼 재구매다. 컨버터가 없어서 지금 만충 운용이 막혀 있다.', { size: 18 }),

      H2('E. USB 허브 계통'),
      flow('배터리 ⊕ ─ 3A 퓨즈 ─▶ XL4015 (5V 5A) ─ DC 플러그 5.5×2.1 ─▶ UH308 ─▶ 라이다·IMU·GPS'),
      itemTable(E, '33,720 ~ 37,720'),
      P('아두이노는 허브에 꽂지 않는다. 허브를 거치면 수신기 전압이 4.09V 까지 떨어진다(최소 4.0V).', { size: 18 }),

      H2('G. 제어 계통 — Arduino Mega'),
      flow('노트북 USB ─▶ Mega 2560 ─┬─▶ 수신기 FS-iA6B        Mega 핀 12·11 ─▶ ESC 좌·우'),
      itemTable(G, '58,000'),
      P('보유 1대는 1번배용이다. 2번배 회로 세팅(10.1~10.15)에 1대가 더 필요하다. 동아리 계획서엔 2대로 올라가 있다.', { size: 18 }),

      H2('주문 전에 확인할 것'),
      bullet('리얼리스토어에 OAK 재고와 발송일을 문의한다. 132만원짜리의 납기를 아직 모른다.'),
      bullet('어댑터는 상세 페이지에서 칩셋이 AX88179 인지 직접 확인한다. 이름이 비슷한 TP-Link UE300C 는 RTL8153 이라 다르다.'),
      bullet('디바이스마트 물건(컨버터·허브·퓨즈)은 한 번에 담는다. 약 47,000원이라 66,000원 무료배송선 미달, 배송비 2,700원.'),
      bullet('2번배 몫 확인: 접촉기·무선 릴레이·비상정지 버튼·수신기·방수 박스·IMU·LED 가 2척분 있는지. 컨버터·허브·XL4015 는 지금 1척분만 잡혀 있다.'),
      bullet('BF303 홀더가 도착하면 5x20 퓨즈를 끼워 본다. 안 맞으면 F-520 클립(110원)을 전선에 납땜한다.'),

      H2('사지 않는 계통'),
      noTable,
      P('보유: T200 6개 · 4S 배터리 5개(예비 1) · ESC 8개(필요 6) · 접촉기·릴레이·버튼 · 노트북 2대 · D455 1대 · RPLIDAR A3 정상 2대.', { size: 18, color: '595959' }),
    ],
  }],
});

const out = process.argv[2];
Packer.toBuffer(doc).then(b => { fs.writeFileSync(out, b); console.log('wrote', out, b.length); });
