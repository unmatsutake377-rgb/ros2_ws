// 소프트웨어 파트(Ⅲ장)만 담은 보고서 — 통합 원고 하나에서 A·B, 제출용·검토용을 만든다.
// 실행: NODE_PATH=<docx 모듈 경로> node tools/make_software_report.js
//   원고: docs/전달용/보고서_분담/통합_Ⅲ소프트웨어.md  (본 보고서 생성기 make_report_docx.js 도 같은 원고를 쓴다)
//   출력: docs/전달용/KABOAT2026_소프트웨어_SSF_{A,B}.docx   (제출용 — [근거] 삭제)
//         docs/전달용/보고서_분담/검토용_소프트웨어_SSF_{A,B}.docx (근거 회색 표시 — 대조용)
const fs = require('fs');
const path = require('path');
const docx = require('docx');
const { Document, Packer, Paragraph, TextRun, HeadingLevel, AlignmentType } = docx;
const { renderMd } = require('./report_md');

const ROOT = path.resolve(__dirname, '..');
const SRC = path.join(ROOT, 'docs/전달용/보고서_분담/통합_Ⅲ소프트웨어.md');
const FIG = path.join(ROOT, 'docs/전달용/배치도');
const FONT = 'Malgun Gothic';
const t = (text, o = {}) => new TextRun({ text, font: FONT, size: 20, ...o });

const styles = { default: { document: { run: { font: FONT, size: 20 } } },
  paragraphStyles: [
    { id: 'Heading1', name: 'Heading 1', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 32, bold: true, font: FONT }, paragraph: { spacing: { after: 200 }, outlineLevel: 0 } },
    { id: 'Heading2', name: 'Heading 2', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 26, bold: true, font: FONT }, paragraph: { outlineLevel: 1 } },
    { id: 'Heading3', name: 'Heading 3', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 22, bold: true, font: FONT }, paragraph: { outlineLevel: 2 } },
    { id: 'Heading4', name: 'Heading 4', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 20, bold: true, font: FONT }, paragraph: { outlineLevel: 3 } } ] };

function cover(boat, review) {
  const md = fs.readFileSync(SRC, 'utf8');
  const toc = md.split('\n').filter(l => /^## /.test(l)).map(l => l.slice(3).trim());
  const out = [
    new Paragraph({ children: [t('2026 KABOAT 자율운항보트 경진대회', { size: 24 })], alignment: AlignmentType.CENTER, spacing: { before: 2400, after: 120 } }),
    new Paragraph({ children: [t('설계보고서 — Ⅲ. 소프트웨어', { bold: true, size: 40 })], alignment: AlignmentType.CENTER, spacing: { after: 200 } }),
    new Paragraph({ children: [t(`S.S.F ${boat}  (${boat === 'A' ? '1번배' : '2번배'})`, { bold: true, size: 28 })], alignment: AlignmentType.CENTER, spacing: { after: 600 } }),
  ];
  if (review) out.push(new Paragraph({ children: [t('검토용 — 회색 [근거: 파일:행] 은 제출본에서 지운다. ____ 는 실측 예정.', { color: '1F7A3A', italics: true, size: 18 })], alignment: AlignmentType.CENTER, spacing: { after: 400 } }));
  out.push(new Paragraph({ children: [t('목차', { bold: true, size: 24 })], spacing: { after: 120 } }));
  for (const h of toc) out.push(new Paragraph({ children: [t(h)], indent: { left: 360 }, spacing: { after: 60 } }));
  return out;
}

(async () => {
  const md = fs.readFileSync(SRC, 'utf8');
  for (const boat of ['A', 'B']) {
    for (const review of [false, true]) {
      const body = renderMd(md, { boat, strip: !review, figDir: FIG, docx, pageBreakH1: true });
      const doc = new Document({ styles, sections: [{ properties: { page: { size: { width: 11906, height: 16838 }, margin: { top: 1440, bottom: 1440, left: 1273, right: 1273 } } }, children: [...cover(boat, review), ...body] }] });
      const out = review ? path.join(ROOT, `docs/전달용/보고서_분담/검토용_소프트웨어_SSF_${boat}.docx`) : path.join(ROOT, `docs/전달용/KABOAT2026_소프트웨어_SSF_${boat}.docx`);
      fs.writeFileSync(out, await Packer.toBuffer(doc));
      console.log('wrote', path.relative(ROOT, out));
    }
  }
})();
