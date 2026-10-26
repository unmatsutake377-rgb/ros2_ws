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
  { name: 'OAK-1 PoE FIXED-FOCUS 69°', use: '[지원금 들어옴] 자율운항 비전 카메라. 마스트 꼭대기에서 부표·표식 인식. 주문 후 수입 상품(통관부호 필요) — 납기 먼저 문의', qty: '2', price: '1,322,000', pri: 'P0', links: [['리얼리스토어', 'https://reallystore.net/goods/view?no=45']] },
  { name: 'PoE 인젝터 REVOTECH PSE5502G 30W DC 12-48V→48V (G마켓)', use: '4S 배터리를 48V PoE 로 변환. 랜선 한 가닥으로 카메라 전원+데이터. 옥션분(10-14) 미발송 → 10-26 G마켓 재발주: 오늘출발·무료배송·국내배송만. 상세 사진으로 모델 PSE5502G·802.3af/at·30W·기가 확인', qty: '2', price: '122,000 (개당 61,000)', pri: 'P0', links: [['G마켓 4578145347', 'http://item.gmarket.co.kr/Item?goodscode=4578145347'], ['대안 하이링크', 'https://search.danawa.com/dsearch.php?query=DIVA-IPoEI']] },
  { name: 'USB-C 기가비트 어댑터 강원전자 NETmate NM-UCL02', use: '인젝터 LAN 과 노트북 연결. 노트북에 랜 포트가 없다. 칩셋 ASIX AX88179 정확히 일치 — 우분투 기본 드라이버, 설치 0. 10-14 선정', qty: '2', price: '36,980 (개당 18,490, 배송 3,000 별도)', pri: 'P0', links: [['다나와 NM-UCL02', 'https://prod.danawa.com/info/?pcode=15885359']] },
  { name: '퓨즈홀더 BF303 + 5x20 퓨즈 2A', use: '인젝터 입력 보호. 배터리 ⊕ 와 인젝터 사이. 퓨즈는 1회용이라 예비 포함 10개. 홀더는 1번배분 1개, 2번배는 실험 후(09-30 원칙)', qty: '홀더 1 · 퓨즈 10', price: '2,400', pri: 'P1', links: [['홀더', DM + '14122158'], ['2A', DM + '4538']] },
];
const B = [
  { name: 'USB 3.0 C↔A 케이블 1m 이하', use: '수동 경기 조종 화면. USB 2.0 케이블이면 0fps. SS 로고·파란 단자 확인, 충전 전용 금지', qty: '2 + 예비 1', price: '15,000~30,000 (개당 5천~1만)', pri: 'P0', links: [['IN-U3TOC 1m', 'https://prod.danawa.com/info/?pcode=16803692'], ['벨킨 F2CU029bt', 'https://prod.danawa.com/info/?pcode=3404024']] },
];
const D = [
  { name: '승압 컨버터 SZH-BPM001 (입력 9~20V → 24V 3A)', use: '비상정지 접촉기 코일 구동. 4S 만충 16.8V 까지 덮는다. 확보 전까지 배터리 16V 초과 충전 금지', qty: '1 + 예비 1', price: '26,400 (개당 13,200)', pri: 'P0', links: [['디바이스마트', DM + '1330749']] },
  { name: 'RF 무선스위치 무선릴레이 + 리모컨', use: '[지원금 들어옴] 2번배 무선 비상정지. 코일 회로에 버튼과 직렬. 1번배(SRD-12VDC-SL-C)와 같은 제품으로', qty: '1', price: '27,200', pri: 'P0', links: [['다나와 검색', 'https://search.danawa.com/dsearch.php?query=RF+%EB%AC%B4%EC%84%A0%EC%8A%A4%EC%9C%84%EC%B9%98+%EB%AC%B4%EC%84%A0%EB%A6%B4%EB%A0%88%EC%9D%B4']] },
];
const E = [
  { name: 'ipTIME UH308 전원형 USB 3.0 허브', use: '라이다·IMU·GPS USB 확장. 아두이노는 여기 꽂지 않고 노트북 직결', qty: '1', price: '27,900', pri: 'P1', links: [['다나와', 'https://prod.danawa.com/info/?pcode=2223672'], ['디바이스마트', DM + '1214178']] },
  { name: 'DC-DC 강압 5V 고정 5A SZWENGAO WG-1224S0505 (IP68) + DC잭 케이블 암수 세트 5.5×2.1', use: '배터리에서 허브용 5V 생성. 10-14 교체 — 메카솔루션 XL4015 가 단종·구매 불가. 5V 고정 출력이라 트리머 조정 없음. 25W = 1.7A 라 3A 퓨즈 그대로', qty: '1 + 1', price: '14,620 (모듈 10,120 VAT포함 + 케이블 4,500)', pri: 'P1', links: [['엘레파츠 모듈', 'https://www.eleparts.co.kr/goods/view?no=19044392'], ['쿠팡 DC잭', 'https://www.coupang.com/vp/products/9590537984?itemId=28629147971&vendorItemId=95599694757']] },
  { name: 'DC 5.5×2.1 수(MALE) 플러그 케이블 30cm', use: '인젝터·허브 DC IN 잭. 배당 2개 = 4개 필요, 티놀지 세트에 수 1개뿐. 인젝터에 리드선 동봉되면 예비. 극성(중심 +) 라벨·멀티미터로 확인 후 삽입', qty: '3', price: '2,970 + 배송 3,000', pri: 'P1', links: [['쿠팡', 'https://www.coupang.com/vp/products/9650750584?itemId=28845278978&vendorItemId=93238400586']] },
  { name: '퓨즈홀더 BF303 + 5x20 퓨즈 3A', use: '허브 급전(5V 모듈 입력) 보호. 퓨즈는 1회용이라 예비 포함 10개. 홀더는 1번배분 1개, 2번배는 실험 후', qty: '홀더 1 · 퓨즈 10', price: '2,400', pri: 'P1', links: [['홀더', DM + '14122158'], ['3A', DM + '4537']] },
];

const G = [
  { name: 'Arduino Mega 2560 R3 이탈리아 정품 (A000067)', use: '[지원금 들어옴] 2번배 메인 제어기. RC 수신·ESC 신호·LED·비상정지 감지. 보유 1대는 1번배용. 10-14 디바이스마트로 확정 — 컨버터·허브·퓨즈와 한 장바구니면 무료배송', qty: '1', price: '64,900 (59,000+VAT)', pri: 'P0', links: [['디바이스마트 no=34405', 'https://www.devicemart.co.kr/goods/view?no=34405']] },
];

const OPTCOLS = [2600, 4200, 1400, 6370];
const oakOptRows = [
  ['Focus 선택', 'AUTO-FOCUS (A00124)', '✗', '렌즈가 전자석으로 움직인다. 배 진동·파도에 초점이 흔들린다. 제조사도 진동 환경엔 고정초점 권장'],
  ['Focus 선택', 'FIXED-FOCUS (OAK-1-PoE-FF)', '✓ 고른다', 'HFOV 69°(AF 66° 보다 넓음), 50cm~∞ 고정. 부표는 전부 50cm 밖. 코드 hfov_deg 가 이 값 기준'],
  ['Focus 선택', '32MP (OAK-1-MAX-PoE) +87,000', '✗', 'HFOV 45° 로 좁아진다. 해상도는 남아돌고 시야만 손해. 2대면 174,000 추가'],
  ['PoE Injector 선택', 'US Cord ; U-POE +42,000', '✗ 빈칸', '110V 벽 콘센트용. 배엔 콘센트 없다. 4S 배터리용 12-48V 인젝터를 옥션에서 따로 산다(위 표)'],
];
const oakOptTable = new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: OPTCOLS, rows: [
  headerRow(OPTCOLS, ['옵션', '선택지', '결론', '이유']),
  ...oakOptRows.map(r => new TableRow({ children: r.map((v, j) => cell(
    para(t(v, { bold: j === 2, color: j === 2 ? (r[2].startsWith('✓') ? '1F7A3A' : 'C00000') : '000000' }),
         { alignment: j === 2 ? AlignmentType.CENTER : AlignmentType.LEFT }),
    OPTCOLS[j], r[2].startsWith('✓') ? { fill: 'E8F3EA' } : {})) })),
] });

const ADCOLS = [3400, 1600, 2000, 7570];
const adRows = [
  ['강원전자 NETmate NM-UCL02', 'AX88179', '18,490', '✓ 고른다 — 칩셋 글자까지 일치, 우분투 기본 드라이버. 국내 브랜드, Linux 명시, 판매처 다수, 리뷰 12'],
  ['강원전자 NETmate NM-UCL01', 'AX88179A', '12,900', '✗ 더 싸지만 칩셋이 A 버전. 같은 드라이버라도 변수를 안 만든다'],
  ['UGREEN CM483', 'AX88179A', '26,900', '✗ A 버전 + 판매처 1곳'],
  ['UGREEN US236', 'AX88179', '26,900', '✗ 칩셋은 맞지만 판매처 1곳, 8,000원 비쌈'],
  ['벤션 C타입 울트라', 'AX88179', '29,900', '✗ 판매처 2곳, 11,000원 비쌈'],
  ['TP-Link UE300C (참고)', 'RTL8153', '—', '✗ 이름이 비슷해 헷갈리기 쉬움. 칩셋이 다르다'],
];
const adTable = new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: ADCOLS, rows: [
  headerRow(ADCOLS, ['후보 (다나와 10-14)', '칩셋', '가격(원)', '결론']),
  ...adRows.map(r => new TableRow({ children: r.map((v, j) => cell(
    para(t(v, { bold: j === 0 || (j === 3 && v.startsWith('✓')), color: j === 3 ? (v.startsWith('✓') ? '1F7A3A' : 'C00000') : '000000' }),
         { alignment: j === 2 ? AlignmentType.RIGHT : AlignmentType.LEFT }),
    ADCOLS[j], r[3].startsWith('✓') ? { fill: 'E8F3EA' } : {})) })),
] });

const F = [
  { name: 'EC5 전원케이블 피그테일 10AWG 100mm 90A (암잭)', use: '배터리 EC5 분기. 접촉기 앞에서 컨버터·인젝터·XL4015 입력을 따는 자리. 10-14 팀 추가. 3개 배정은 회로팀 확정', qty: '3', price: '9,000 (개당 3,000)', pri: 'P0', links: [['쿠팡 로켓', 'https://www.coupang.com/vp/products/8713579160?itemId=25306396421&vendorItemId=92301572146']] },
];

const ARRCOLS = [3000, 900, 900, 5200, 4570];
const arrRows = [
  ['OAK-1 PoE FF', '2', '2', '로젠 453-9802-8546, 리얼리, 10-19 접수', '✓ 도착'],
  ['Arduino Mega 2560 R3', '1', '1', '롯데 3171-3951-2406, 엔티렉스(디바이스마트), A000067 정품', '✓ 도착'],
  ['승압 컨버터 SZH-BPM001', '2', '2', '롯데 3171-3951-2395, 2 EA 26,400', '✓ 도착'],
  ['ipTIME UH308 허브', '1', '1', '린드스토어 5121-7531-5564', '✓ 도착'],
  ['NETmate NM-UCL02 어댑터', '2', '2', '사진 2봉 — 노트북 USB-C 직결', '✓ 도착'],
  ['USB 3.0 C↔A 케이블', '2+1', '3', 'CJ 6009-5562-0923, 디지월드, 벨킨 F2CU029 10Gbps 1m ×3', '✓ 도착'],
  ['SZWENGAO 5V 5A 모듈', '1', '2', '엘레파츠 453-8832-4030, 사진 2개', '✓ 도착 (1개 초과, 2번배 예비)'],
  ['퓨즈 5x20 2A', '10', '10', '롯데 3171-3951-2384, 10 EA', '✓ 도착'],
  ['퓨즈홀더 BF303', '2', '2', '디바이스마트 봉투 2 EA — 1번배분, 2번배는 실험 후', '✓ 도착'],
  ['퓨즈 5x20 3A', '10', '0', '사진 없음', '⚠ 미확인'],
  ['PoE 인젝터', '2', '0', '옥션분 미발송 → 10-26 G마켓 61,000 재발주(오늘출발)', '⚠ 재발주'],
  ['RF 무선스위치', '1', '1', '팀 확인 10-22', '✓ 도착'],
  ['EC5 피그테일 (쿠팡)', '3', '3', '팀 확인 10-26', '✓ 도착'],
  ['DC잭 케이블 세트 (쿠팡)', '1', '0', '사진 없음 (엘레파츠 "외" 일 수도)', '⚠ 미확인'],
];
const arrTable = new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: ARRCOLS, rows: [
  headerRow(ARRCOLS, ['품목', '주문', '도착', '송장·라벨 근거', '상태']),
  ...arrRows.map(r => new TableRow({ children: r.map((v, j) => cell(
    para(t(v, { bold: j === 0 || j === 4, color: j === 4 ? (v.startsWith('✓') ? '1F7A3A' : 'C00000') : '000000' }),
         { alignment: (j === 1 || j === 2) ? AlignmentType.CENTER : AlignmentType.LEFT }),
    ARRCOLS[j], r[4].startsWith('✓') ? { fill: 'E8F3EA' } : { fill: 'FDECEA' })) })),
] });

const SUMCOLS = [5000, 2000, 3500, 4070];
const sumRows = [
  ['A. OAK 카메라', '4', '1,483,380', 'P0 카메라·인젝터·어댑터, P1 퓨즈'],
  ['B. D455 조종 카메라', '1', '15,000 ~ 30,000', 'P0'],
  ['C. 라이다', '0', '0', 'A3 1대 10-07 구매 완료(17만원) — 예비 0'],
  ['D. 비상정지', '2', '53,600', 'P0 — 컨버터 + 2번배 무선스위치(지원금)'],
  ['E. USB 허브', '4', '47,890', 'P1 (+DC 수 플러그 ×3)'],
  ['F. 주 전력 (EC5)', '1', '9,000', 'P0 — 분기용 피그테일 ×3'],
  ['G. 제어 (Mega)', '1', '64,900', 'P0 — 2번배용. 지원금 들어옴'],
  ['합계', '12', '1,673,770 ~ 1,688,770', 'OAK 2대가 78~79%'],
];
const sumTable = new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: SUMCOLS, rows: [
  headerRow(SUMCOLS, ['계통', '품목 수', '금액(원)', '비고']),
  ...sumRows.map((r, i) => new TableRow({ children: r.map((v, j) => cell(
    para(t(v, { bold: i === sumRows.length - 1 || j === 0 }), { alignment: j === 1 ? AlignmentType.CENTER : j === 2 ? AlignmentType.RIGHT : AlignmentType.LEFT }),
    SUMCOLS[j], i === sumRows.length - 1 ? { fill: 'F2F2F2' } : {})) })),
] });

const NOCOLS = [3000, 5500, 6070];
const noRows = [
  ['C. 라이다', 'A3 1대 10-07 구매 완료 — 정상품 2대(80BF·918A)', '예비 0(D5F1 소재 미확인). 2번배 udev 포트 고정(F7). 단선품에 "쓰지 말 것" 표시'],
  ['F. 주 전력 본선', '08-25 재배선 끝, 09-07 물 위 시험 통과 (분기 피그테일만 위 F 에서 구매)', '인젝터·허브 급전용 가는 선(22~26AWG)은 재고 확인'],
  ['G. 제어 전원', '노트북을 항상 싣는다 — Mega·수신기는 노트북 USB (Mega 본체는 위 G 에서 구매)', 'LED 밝기 40 한낮 시인성 시험. 안 보이면 UBEC 5V 3A 구매(접촉기 앞에서 분기)'],
  ['H. 마스트·기구', '필라멘트·나사 재고 또는 외부 출력', '관통부는 실란트 마감 + 드립 루프'],
  ['I. GPS', '접지판은 자체 제작', '알루미늄 Ø100~150mm, 나사 2개 고정 (RTK FIXED 조건)'],
  ['J. 배 ID', '분기에 쓰는 곳이 없다 (09-17 전수 확인)', '없음. 두 배 회로 동일'],
];
const noTable = new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: NOCOLS, rows: [
  headerRow(NOCOLS, ['계통', '왜 안 사나', '남은 일'], '595959'),
  ...noRows.map(r => new TableRow({ children: r.map((v, j) => cell(para(t(v, { bold: j === 0 })), NOCOLS[j])) })),
] });

// ---- 금액 정리 (지원금 / 자부담)
const MCOLS = [4200, 3800, 6570];
function moneyTable(rows, head, fill, totalLabel) {
  return new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: MCOLS, rows: [
    headerRow(MCOLS, head, fill),
    ...rows.map((r, i) => {
      const last = totalLabel && i === rows.length - 1;
      return new TableRow({ children: r.map((v, j) => cell(
        para(t(v, { bold: last || j === 0 }), { alignment: j === 1 ? AlignmentType.RIGHT : AlignmentType.LEFT }),
        MCOLS[j], last ? { fill: 'F2F2F2' } : {})) });
    }),
  ] });
}
const BIG = [2600, 4200, 7770];
const bigRow = (a, b, c, color) => new TableRow({ children: [
  cell(para(t(a, { bold: true, size: 22 })), BIG[0]),
  cell(para(t(b, { bold: true, size: 24, color }), { alignment: AlignmentType.RIGHT }), BIG[1]),
  cell(para(t(c)), BIG[2]),
] });
const bigTable = new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: BIG, rows: [
  headerRow(BIG, ['구분', '금액(원, VAT 포함)', '내용']),
  bigRow('총 사야 하는 금액', '1,673,770 ~ 1,688,770', '12항목 전체', '000000'),
  bigRow('지원금', '1,414,100', 'OAK-1 PoE ×2 · Arduino Mega ×1 · RF 무선스위치 ×1 (동아리 2차 활동계획서)', '1F7A3A'),
  bigRow('자부담 (지원금 제외)', '259,670 ~ 274,670', '나머지 9항목 — 우리가 내야 하는 금액', 'C00000'),
] });
const fundRows = [
  ['OAK-1 PoE ×2', '1,322,000', '계획서 7번'],
  ['Arduino Mega 2560 R3 ×1', '64,900', '계획서 3번 — 승인 119,200(2대) 안. 필요 1대'],
  ['RF 무선스위치 무선릴레이 ×1', '27,200', '계획서 11번'],
  ['지원금 합계', '1,414,100', ''],
];
const selfRows = [
  ['PoE 인젝터 30W 12-48V (G마켓) ×2', '122,000', 'A. OAK 카메라'],
  ['USB-C 어댑터 NM-UCL02 ×2', '36,980', 'A. OAK 카메라'],
  ['USB 3.0 C↔A 케이블 ×3', '15,000 ~ 30,000', 'B. D455'],
  ['승압 컨버터 SZH-BPM001 ×2', '26,400', 'D. 비상정지'],
  ['ipTIME UH308 허브 ×1', '27,900', 'E. USB 허브'],
  ['5V 고정 강압 모듈 + DC잭 세트', '14,620', 'E. USB 허브'],
  ['DC 5.5×2.1 수 플러그 ×3', '2,970', 'E. USB 허브 / A 인젝터'],
  ['퓨즈홀더 ×2 + 퓨즈 2A×10·3A×10', '4,800', 'A·E'],
  ['EC5 피그테일 ×3', '9,000', 'F. 주 전력'],
  ['자부담 합계', '259,670 ~ 274,670', ''],
];

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
      P('2026-10-22 기준 · 단일 출처: 저장소 docs/전달용/구매_목록.md · 가격은 09-14~09-23 다나와·공식샵 기준 추정 · 금액 = 적힌 수량 전체, VAT 포함', { color: '595959', size: 17 }),
      P('사야 할 것은 12항목, 합계 약 168~169만원이다. 아직 하나도 주문하지 않았다. 같은 계통 안에서 하나만 빠져도 그 계통은 안 돈다.', { bold: true }),
      sumTable,

      H2('도착 현황 (2026-10-22, 송장·라벨 사진 대조)'),
      P('주문 10-18~19, 도착 10-20~22. 12항목 중 10항목 도착, 미도착 4(인젝터 재발주·DC잭·수 플러그·3A 퓨즈).', { bold: true }),
      arrTable,
      P('도착분으로 되는 것: 비상정지 D 는 컨버터 결선하면 만충 운용 가능. 카메라 A 는 인젝터만 오면 완성. 허브 E 는 3A 퓨즈·DC잭만 오면 완성.', { size: 18 }),

      H2('A. OAK 카메라 계통 — 자율운항 비전'),
      flow('배터리 ⊕ ─ 2A 퓨즈 ─▶ PoE 인젝터 ─ 동봉 2m Cat5e ─▶ OAK-1 PoE      인젝터 LAN ─▶ NM-UCL02(C 플러그) ─▶ 노트북 USB-C 직결'),
      itemTable(A, '1,483,380'),
      P('OAK-1 PoE 주문 옵션 (리얼리스토어 페이지 10-14 확인)', { bold: true }),
      oakOptTable,
      P('USB-C 기가비트 어댑터 후보 비교 — NM-UCL02 선정 (10-14)', { bold: true }),
      adTable,
      P('랜선(Cat5e 2m)은 OAK 동봉이라 사지 않는다. 인젝터 전원은 접촉기 「앞」에서 딴다 — 비상정지는 추진만 끊는다.', { size: 18 }),

      H2('B. D455 조종 카메라 계통 — 수동 경기 화면'),
      flow('D455 ─ USB 3.0 C↔A (1m 이하) ─▶ 노트북'),
      itemTable(B, '15,000 ~ 30,000'),
      P('D455 는 1대 보유. 배마다 1대씩 쓰면 1대 추가 구매가 필요하다 — 미정.', { size: 18 }),

      H2('C. 라이다 계통 — A3 1대 구매 완료 (10-07, 17만원 중고)'),
      P('시리얼 918A9AF2, FW 1.25, 1612점/0.223° 로 셋 중 가장 정밀. 10-07 실측으로 정상품 2대(80BF=1번배 · 918A=2번배). D5F1 은 소재 미확인이라 예비는 여전히 0. 딸려온 USB 어댑터 시리얼이 공장값 0001 이라 2번배 노트북은 udev 포트 고정(F7).', { size: 18 }),

      H2('D. 비상정지 계통'),
      flow('배터리A ⊕ (접촉기 앞) ─▶ 승압 컨버터 ─▶ 코일 A1·A2 ─▶ 무선 릴레이 COM/NC ─▶ 버튼 NC'),
      itemTable(D, '53,600'),
      P('접촉기·버튼은 보유. 무선스위치는 1번배용 1대뿐이라 2번배용 1대 추가. 08-25 주문분이 09-17 취소돼 재구매다. 컨버터가 없어서 지금 만충 운용이 막혀 있다.', { size: 18 }),

      H2('E. USB 허브 계통'),
      flow('배터리 ⊕ ─ 3A 퓨즈 ─▶ SZWENGAO 5V 고정 5A (IP68) ─ DC 플러그 5.5×2.1 ─▶ UH308 ─▶ 라이다·IMU·GPS'),
      itemTable(E, '47,890'),
      P('아두이노는 허브에 꽂지 않는다. 허브를 거치면 수신기 전압이 4.09V 까지 떨어진다(최소 4.0V).', { size: 18 }),

      H2('F. 주 전력 계통 — EC5 분기 피그테일'),
      flow('배터리A EC5 ─┬─▶ (기존) 접촉기 5번 ─▶ ESC        └─ EC5 암잭 피그테일 ─▶ 접촉기 앞 분기: 컨버터·인젝터·XL4015 IN+'),
      itemTable(F, '9,000'),
      P('본선(배터리↔ESC)은 08-25 에 끝났다. 이건 접촉기 앞에서 보조 전원을 따기 위한 분기 커넥터다. 옵션은 암잭 — 배터리 쪽 EC5 수컷에 꽂는다.', { size: 18 }),

      H2('G. 제어 계통 — Arduino Mega'),
      flow('노트북 USB ─▶ Mega 2560 ─┬─▶ 수신기 FS-iA6B        Mega 핀 12·11 ─▶ ESC 좌·우'),
      itemTable(G, '64,900'),
      P('보유 1대는 1번배용이다. 2번배 회로 세팅(10.1~10.15)에 1대가 더 필요하다. 동아리 계획서엔 2대로 올라가 있다.', { size: 18 }),

      H2('주문 전에 확인할 것'),
      bullet('리얼리스토어 031-8091-0707 (평일 09~18) 에 납기 문의. 페이지에 "해외에서 수입하여 판매" + 통관부호 수집 — 국내 재고가 아니라 주문 후 수입이다. 답이 2주 이상이어도 바로 발주한다.'),
      bullet('어댑터는 NM-UCL02 로 확정(칩셋 AX88179). 다른 걸 살 땐 상세 페이지 칩셋을 직접 확인 — UE300C 는 RTL8153, NM-UCL01 은 AX88179A.'),
      bullet('디바이스마트 물건(컨버터·Mega·허브·퓨즈)은 한 번에 담는다. 126,600원이라 66,000원 넘겨 무료배송.'),
      bullet('2번배 몫은 1번배로 실험한 뒤 추가 구매한다: 컨버터·허브·XL4015(지금 1척분), 접촉기·비상정지 버튼·수신기·방수 박스·IMU·LED(보유 대수 미확인).'),
      bullet('BF303 홀더가 도착하면 5x20 퓨즈를 끼워 본다. 안 맞으면 F-520 클립(110원)을 전선에 납땜한다. 예비 퓨즈는 배에 항상 2A·3A 몇 개씩 싣는다.'),

      H2('사지 않는 계통'),
      noTable,
      P('보유: T200 6개 · 4S 배터리 5개(예비 1) · ESC 8개(필요 6) · 접촉기·릴레이·버튼 · 노트북 2대 · D455 1대 · RPLIDAR A3 정상 2대(80BF=1번배 · 918A=2번배).', { size: 18, color: '595959' }),

      H2('금액 정리 — 지원금과 자부담'),
      bigTable,
      P('지원금 품목', { bold: true }),
      moneyTable(fundRows, ['품목', '금액(원)', '근거'], '1F7A3A', true),
      P('자부담 품목 — 지원금에 없는 것', { bold: true }),
      moneyTable(selfRows, ['품목', '금액(원)', '계통'], 'C00000', true),
      P('금액은 품목가 기준이고 배송비는 빠져 있다. 계획서 기준(배송 포함, Mega 2대)으로는 지원 품목이 1,474,900원이다. 계획서 전체 총계는 2,214,500원.', { size: 18, color: '595959' }),
      P('2번배 몫(컨버터·허브·XL4015 추가분, 접촉기·버튼·수신기·방수 박스·IMU·LED)은 1번배 실험 뒤 추가 구매라 여기 들어 있지 않다. 추가분은 다음 회차 지원금에 넣지 못하면 자부담이다.', { size: 18, color: '595959' }),
    ],
  }],
});

const out = process.argv[2];
Packer.toBuffer(doc).then(b => { fs.writeFileSync(out, b); console.log('wrote', out, b.length); });
