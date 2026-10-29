// 보고서 마크다운 → docx 요소 렌더러 (본 보고서 생성기와 같은 스타일)
//   const { renderMd } = require('./report_md');
//   renderMd(mdText, { boat: 'A'|'B', strip: true|false, figDir, docx })  → Paragraph/Table 배열
// 문법: # H1 · ## H2 · ### H3 · #### H4 · '- ' 글머리 · **굵게** · `코드`(Consolas) · [근거: …](strip 이면 삭제, 아니면 회색)
//       파이프 표 · ![캡션](파일.png) 그림(figDir) · '그림: 설명' 한 줄 = 그림 자리 박스
// 배별 자리표시 {{…}} 는 BOAT_TOKENS 로 치환한다(없는 토큰이면 예외 — 조용히 남기지 않는다).
const fs = require('fs');
const path = require('path');

const BOAT_TOKENS = {
  A: {
    BOAT: 'A', BOAT_NO: '1번배', ESC_FANOUT: '측당 ESC 1개',
    LIDAR: 'RPLIDAR A3 개체 80BF (약 11.1 Hz / 0.250°)',
    GPS: 'u-blox ZED-F9P (1번배 실물 확정 10-29, 안테나 ⚠️ 확인)', GPS_CFG: 'zed_f9p_rover.yaml(기본값)',
    HALF_WIDTH_REASON: '1번배 폭 0.57 m → 0.285, 공용값 0.29',
  },
  B: {
    BOAT: 'B', BOAT_NO: '2번배', ESC_FANOUT: '측당 ESC 2개 팬아웃',
    LIDAR: 'RPLIDAR A3 개체 918A (11.0 Hz / 0.223°)',
    GPS: 'u-blox C94-M8P 또는 ZED-F9P — ⚠️ 2번배 수신기 미정', GPS_CFG: '⚠️ 수신기 확정 후(M8P 면 c94_m8p_rover.yaml)',
    HALF_WIDTH_REASON: '2번배 폭 0.535 m → 0.2675, 공용값 0.29(시뮬 접촉 0.95·정체 0)',
  },
};

function substitute(md, boat) {
  const tok = BOAT_TOKENS[boat];
  if (!tok) throw new Error('boat 는 A 또는 B: ' + boat);
  return md.replace(/\{\{([A-Z_]+)\}\}/g, (m, k) => {
    if (!(k in tok)) throw new Error('모르는 자리표시: ' + m);
    return tok[k];
  });
}

function renderMd(mdRaw, opt) {
  const D = opt.docx;
  const { Paragraph, TextRun, ImageRun, Table, TableRow, TableCell, WidthType, ShadingType, HeadingLevel, AlignmentType, BorderStyle, PageBreak } = D;
  const FONT = 'Malgun Gothic'; const TW = 9360; const STRIP = !!opt.strip;
  const b = { style: BorderStyle.SINGLE, size: 6, color: '000000' }; const borders = { top: b, bottom: b, left: b, right: b };
  const t = (text, o = {}) => new TextRun({ text, font: FONT, size: 20, ...o });
  function runs(text, base = {}) {
    const out = []; const re = /(\*\*[^*]+\*\*|\[근거:[^\]]*\]|`[^`]+`)/g; let last = 0; let m;
    while ((m = re.exec(text))) {
      if (m.index > last) out.push(t(text.slice(last, m.index), base));
      const tok = m[0];
      if (tok.startsWith('**')) out.push(...runs(tok.slice(2, -2), { ...base, bold: true }));
      else if (tok.startsWith('`')) out.push(new TextRun({ text: tok.slice(1, -1), font: 'Consolas', size: (base.size || 20) - 2, bold: base.bold }));
      else if (!STRIP) out.push(t(tok, { ...base, size: 16, color: '7F7F7F' }));
      last = m.index + tok.length;
    }
    if (last < text.length) out.push(t(text.slice(last), base));
    // 근거를 지운 뒤 남는 '문장 끝 공백 + 마침표' 정리
    return out.length ? out : [t('')];
  }
  const clean = (s) => STRIP ? s.replace(/\s*\[근거:[^\]]*\]/g, '').replace(/\s+([.,)])/g, '$1') : s;
  const P = (s) => new Paragraph({ children: runs(clean(s)), spacing: { after: 100, line: 300 } });
  const B = (s) => new Paragraph({ children: [t('•  '), ...runs(clean(s))], spacing: { after: 60, line: 290 }, indent: { left: 360, hanging: 240 } });
  const H1 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_1, children: [...(opt.pageBreakH1 === false ? [] : [new PageBreak()]), t(s, { bold: true, size: 32 })], spacing: { before: 0, after: 200 } });
  const H2 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_2, children: [t(s, { bold: true, size: 26 })], spacing: { before: 240, after: 120 } });
  const H3 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_3, children: [t(s, { bold: true, size: 22 })], spacing: { before: 160, after: 80 } });
  const H4 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_4, children: runs(clean(s), { bold: true }), spacing: { before: 140, after: 60 } });
  const CAP = (s) => new Paragraph({ children: [t(s, { size: 18, color: '555555' })], alignment: AlignmentType.CENTER, spacing: { after: 160 } });
  const SP = () => new Paragraph({ children: [t('')], spacing: { after: 60 } });
  const BOX = (s) => new Paragraph({ children: [t('[그림 자리] ' + s, { color: '888888' })], alignment: AlignmentType.CENTER, spacing: { before: 200, after: 200 }, border: { top: b, bottom: b, left: b, right: b } });
  const cell = (ch, w, fill) => new TableCell({ width: { size: w, type: WidthType.DXA }, borders, margins: { top: 60, bottom: 60, left: 90, right: 90 }, shading: fill ? { fill, type: ShadingType.CLEAR, color: 'auto' } : undefined, children: [ch] });
  function table(rows) {
    const head = rows[0]; const n = head.length;
    // 열 너비: 머리글·내용 글자 수 비례(최소 1, 최대 4 배)
    const len = head.map((_, j) => Math.max(...rows.map(r => clean(r[j] || '').replace(/\[근거:[^\]]*\]|`|\*\*/g, '').length), 2));
    const lo = Math.min(...len); const wr = len.map(x => Math.min(Math.max(x / lo, 1), 4));
    const s = wr.reduce((a, c) => a + c, 0); let w = wr.map(x => Math.round(x * TW / s));
    // 최소 열 너비 1000 dxa(≈1.8 cm) — 짧은 열의 머리글이 글자 단위로 줄바꿈되지 않게
    const MIN = Math.min(1000, Math.floor(TW / n)); const deficit = w.reduce((a, x) => a + Math.max(0, MIN - x), 0);
    if (deficit > 0) { const big = w.reduce((a, x) => a + (x > MIN ? x - MIN : 0), 0); w = w.map(x => x < MIN ? MIN : Math.round(x - (x - MIN) * deficit / big)); }
    const hr = new TableRow({ tableHeader: true, children: head.map((h, i) => cell(new Paragraph({ children: runs(clean(h), { bold: true }), alignment: AlignmentType.CENTER }), w[i], 'E7E6E6')) });
    const rs = rows.slice(1).map(r => new TableRow({ children: head.map((_, j) => cell(new Paragraph({ children: runs(clean(r[j] || '')) }), w[j])) }));
    return new Table({ width: { size: TW, type: WidthType.DXA }, columnWidths: w, rows: [hr, ...rs] });
  }
  function img(file, cap) {
    const p = path.join(opt.figDir, file);
    if (!fs.existsSync(p)) return [BOX(cap + ' (파일 없음: ' + file + ')'), CAP(cap)];
    const buf = fs.readFileSync(p);
    const W = buf.readUInt32BE(16); const Hh = buf.readUInt32BE(20);   // PNG IHDR
    const sc = Math.min(1, 560 / W, 760 / Hh);
    return [new Paragraph({ children: [new ImageRun({ type: 'png', data: buf, transformation: { width: Math.round(W * sc), height: Math.round(Hh * sc) } })], alignment: AlignmentType.CENTER, spacing: { before: 120, after: 60 } }), CAP(cap)];
  }

  const md = substitute(mdRaw, opt.boat);
  const out = []; let tb = null;
  const flush = () => { if (tb) { out.push(table(tb), SP()); tb = null; } };
  for (const line of md.split('\n')) {
    if (line.startsWith('|')) {
      if (/^\|[\s:|-]+\|\s*$/.test(line)) continue;
      (tb = tb || []).push(line.replace(/^\|\s*|\s*\|\s*$/g, '').split(/\s*\|\s*/)); continue;
    }
    flush();
    if (!line.trim()) continue;
    let m;
    if ((m = line.match(/^# (.*)$/))) out.push(H1(m[1].trim()));
    else if ((m = line.match(/^## (.*)$/))) out.push(H2(m[1].trim()));
    else if ((m = line.match(/^### (.*)$/))) out.push(H3(m[1].trim()));
    else if ((m = line.match(/^#### (.*)$/))) out.push(H4(m[1].trim()));
    else if ((m = line.match(/^!\[([^\]]*)\]\(([^)]+)\)\s*$/))) out.push(...img(m[2].trim(), m[1].trim()));
    else if ((m = line.match(/^그림:\s*(.*)$/))) out.push(BOX(clean(m[1])), CAP(clean(m[1])));
    else if ((m = line.match(/^\s*[-*] (.*)$/))) out.push(B(m[1]));
    else if (/^>/.test(line)) continue;      // 인용(작성 메모)은 보고서에 넣지 않는다
    else out.push(P(line.trim()));
  }
  flush();
  return out;
}

module.exports = { renderMd, substitute, BOAT_TOKENS };
