const fs = require('fs');
const { Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell, WidthType, ShadingType, HeadingLevel, AlignmentType, PageOrientation, ExternalHyperlink, BorderStyle } = require('docx');
const FONT = 'Malgun Gothic'; const TW = 14570;
const b = { style: BorderStyle.SINGLE, size: 4, color: 'A6A6A6' }; const borders = { top: b, bottom: b, left: b, right: b };
const t = (text, o = {}) => new TextRun({ text, font: FONT, size: 20, ...o });
const link = (l, u) => new ExternalHyperlink({ link: u, children: [new TextRun({ text: l, font: FONT, size: 20, style: 'Hyperlink', color: '0563C1', underline: {} })] });
const cell = (ch, w, fill) => new TableCell({ width: { size: w, type: WidthType.DXA }, borders, margins: { top: 70, bottom: 70, left: 110, right: 110 }, shading: fill ? { fill, type: ShadingType.CLEAR, color: 'auto' } : undefined, children: Array.isArray(ch) ? ch : [ch] });
const para = (r, o = {}) => new Paragraph({ children: Array.isArray(r) ? r : [r], ...o });
const COLS = [3600, 900, 1500, 1500, 3570, 3500];
const head = ['품목', '수량', '가격(원)', '구매처', '상태', '링크'];
const rows = [
  ['PoE 인젝터 PSE5502G\n(30W, DC 12-48V→48V, 802.3af/at, 기가)', '2', '61,000 ×2\n= 122,000', 'G마켓', '재발주 — 옥션분(10-14) 미발송 취소. 오후 6시 전 주문 시 오늘출발', ['G마켓 4578145347', 'http://item.gmarket.co.kr/Item?goodscode=4578145347']],
  ['DC잭 케이블 암수 세트 5.5×2.1 15cm (티놀지)', '1', '4,500', '쿠팡', '미도착 — 엘레파츠 박스 "외" 에 들었는지 먼저 확인', ['쿠팡 9590537984', 'https://www.coupang.com/vp/products/9590537984?itemId=28629147971&vendorItemId=95599694757']],
  ['퓨즈 5×20 3A', '10', '110 ×10\n= 1,100', '디바이스마트', '미도착 — 2A 10개는 도착. 같은 주문이면 봉투 하나 더 있는지', ['디바이스마트 4537', 'https://www.devicemart.co.kr/goods/view?no=4537']],
];
const table = new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: COLS, rows: [
  new TableRow({ tableHeader: true, children: head.map((h, i) => cell(para(t(h, { bold: true, color: 'FFFFFF' })), COLS[i], '1F3864')) }),
  ...rows.map(r => new TableRow({ children: r.map((v, j) => {
    if (j === 5) return cell(para(link(v[0], v[1])), COLS[j]);
    const lines = String(v).split('\n');
    return cell(lines.map(l => para(t(l, { bold: j === 0 }), { alignment: j === 1 || j === 2 ? AlignmentType.CENTER : AlignmentType.LEFT })), COLS[j], j === 4 && /재발주/.test(v) ? 'FDECEA' : undefined);
  }) })),
] });
const doc = new Document({
  styles: { default: { document: { run: { font: FONT, size: 20 } } },
    paragraphStyles: [{ id: 'Heading1', name: 'Heading 1', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 30, bold: true, font: FONT, color: '1F3864' }, paragraph: { spacing: { after: 120 } } }] },
  sections: [{ properties: { page: { size: { width: 11906, height: 16838, orientation: PageOrientation.LANDSCAPE }, margin: { top: 1000, bottom: 1000, left: 1134, right: 1134 } } },
    children: [
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun({ text: 'SSF 2026 미배송 · 재발주 목록', font: FONT })] }),
      para(t('2026-10-26 기준 · 12항목 중 10항목 도착. 아래 3건만 남음. DC 5.5×2.1 수 플러그 ×3 은 원래 목록에 없던 추가분이라 제외.', { color: '595959', size: 18 }), { spacing: { after: 160 } }),
      table,
      para(t('막는 것은 인젝터 하나 — OAK 는 PoE 로만 켜진다. 나머지 둘은 허브 계통(E) 마무리용.', { size: 18 }), { spacing: { before: 160 } }),
      para(t('합계 127,600원 (전부 자부담). 전체 구매 목록은 docs/전달용/구매_목록.md §0-1.', { size: 18, color: '595959' })),
    ] }],
});
Packer.toBuffer(doc).then(buf => { fs.writeFileSync(process.argv[2], buf); console.log('wrote', process.argv[2]); });
