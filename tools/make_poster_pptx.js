// A0 포스터 생성기 — 발표자료 12장을 주최측 포스터 양식의 12칸(4행×3열, 칸 = 10×7.5 in = 슬라이드와 같은 크기)에 한 장씩 넣는다.
// 실행: NODE_PATH=<pptxgenjs 모듈 경로(jszip 포함)> node tools/make_poster_pptx.js <발표자료.pptx> <포스터양식.pptx> <출력.pptx> <A|B>
// 방법: 그림으로 바꾸지 않고 슬라이드의 도형·표·그림을 그대로 복사해 칸 위치만큼 좌표를 옮긴다. 글자는 그대로 편집 가능.
// (그룹으로 묶지 않는다 — 표는 PowerPoint 그룹에 못 들어간다)
'use strict';
const fs = require('fs');
const JSZip = require('jszip');

const [, , DECK, TEMPLATE, OUT, BOAT] = process.argv;
if (!DECK || !TEMPLATE || !OUT || !['A', 'B'].includes(BOAT)) {
  console.error('사용법: node tools/make_poster_pptx.js <발표자료.pptx> <포스터양식.pptx> <출력.pptx> <A|B>');
  process.exit(1);
}
const TEAM = BOAT === 'A'
  ? { name: 'S.S.F A', members: '도건윤, 김은우, 김정한, 김하은, 박주현, 황준연, 석현, 정우성' }
  : { name: 'S.S.F B', members: '박유진, 박윤서, 차준혁, 이송언, 이태훈, 김세연, 김현지, 전홍윤, 황예진' };
const SCHOOL = '홍익대학교 조선해양모빌리티공학과';

// 발표자료 테마 색 (tools/make_slides_pptx.js THEME 와 같아야 한다) — 포스터 테마가 다르므로 복사하면서 고정색으로 바꾼다
const DECK_THEME = { dk1: '000000', lt1: 'FFFFFF', dk2: '052E65', lt2: 'F2F2F2',
  accent1: '213A62', accent2: 'C0392B', accent3: '2E7D32', accent4: '7A4B00', accent5: 'FFF5C2', accent6: '666666' };
const SCHEME_ALIAS = { tx1: 'dk1', tx2: 'dk2', bg1: 'lt1', bg2: 'lt2' };
const FONT = '맑은 고딕';
const EMU_IN = 914400;
const SLIDE_W = 10 * EMU_IN, SLIDE_H = 7.5 * EMU_IN;

const xmlEsc = (t) => String(t).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');

// ── 색: schemeClr(+lumMod/lumOff/shade/tint) → srgbClr ──────────────────────────
function hexToRgb(h) { return [0, 2, 4].map((i) => parseInt(h.slice(i, i + 2), 16) / 255); }
function rgbToHex(r, g, b) { return [r, g, b].map((v) => Math.max(0, Math.min(255, Math.round(v * 255))).toString(16).padStart(2, '0')).join('').toUpperCase(); }
function rgbToHsl(r, g, b) {
  const max = Math.max(r, g, b), min = Math.min(r, g, b); let h = 0, s = 0; const l = (max + min) / 2;
  if (max !== min) {
    const d = max - min; s = l > 0.5 ? d / (2 - max - min) : d / (max + min);
    if (max === r) h = (g - b) / d + (g < b ? 6 : 0); else if (max === g) h = (b - r) / d + 2; else h = (r - g) / d + 4;
    h /= 6;
  }
  return [h, s, l];
}
function hslToRgb(h, s, l) {
  if (s === 0) return [l, l, l];
  const q = l < 0.5 ? l * (1 + s) : l + s - l * s, p = 2 * l - q;
  const f = (t) => { if (t < 0) t += 1; if (t > 1) t -= 1; if (t < 1 / 6) return p + (q - p) * 6 * t; if (t < 1 / 2) return q; if (t < 2 / 3) return p + (q - p) * (2 / 3 - t) * 6; return p; };
  return [f(h + 1 / 3), f(h), f(h - 1 / 3)];
}
function applyMods(hex, mods) {
  let [r, g, b] = hexToRgb(hex);
  let [h, s, l] = rgbToHsl(r, g, b);
  for (const [k, v] of mods) {
    const f = v / 100000;
    if (k === 'lumMod') l *= f; else if (k === 'lumOff') l += f; else if (k === 'satMod') s = Math.min(1, s * f);
    else if (k === 'shade') { [r, g, b] = hslToRgb(h, s, l).map((c) => c * f); [h, s, l] = rgbToHsl(r, g, b); }
    else if (k === 'tint') { [r, g, b] = hslToRgb(h, s, l).map((c) => 1 - (1 - c) * f); [h, s, l] = rgbToHsl(r, g, b); }
  }
  l = Math.max(0, Math.min(1, l));
  return rgbToHex(...hslToRgb(h, s, l));
}
function fixSchemeColors(xml) {
  return xml.replace(/<a:schemeClr val="(\w+)"(?:\/>|>(.*?)<\/a:schemeClr>)/gs, (m, val, inner) => {
    const key = SCHEME_ALIAS[val] || val;
    const base = DECK_THEME[key];
    if (!base) return m;               // phClr 등은 그대로
    const mods = [];
    if (inner) for (const mm of inner.matchAll(/<a:(lumMod|lumOff|shade|tint|satMod) val="(\d+)"\/>/g)) mods.push([mm[1], Number(mm[2])]);
    const extra = inner ? inner.replace(/<a:(lumMod|lumOff|shade|tint|satMod) val="\d+"\/>/g, '') : '';   // alpha 등은 남긴다
    return extra.trim() ? `<a:srgbClr val="${applyMods(base, mods)}">${extra}</a:srgbClr>` : `<a:srgbClr val="${applyMods(base, mods)}"/>`;
  });
}

// ── 글꼴: latin/ea 없는 rPr 에 맑은 고딕 지정 (포스터 테마 글꼴이 Corbel 이라) ─────
function fixFonts(xml) {
  return xml.replace(/<a:(rPr|endParaRPr|defRPr)\b([^>]*?)(\/>|>([\s\S]*?)<\/a:\1>)/g, (m, tag, attrs, close, inner) => {
    const body = inner || '';
    if (/<a:latin\b/.test(body)) return m;
    const add = `<a:latin typeface="${FONT}"/><a:ea typeface="${FONT}"/>`;
    return `<a:${tag}${attrs}>${body}${add}</a:${tag}>`;
  });
}

// ── 슬라이드 하나에서 복사할 도형 XML 목록 ───────────────────────────────────────
function topLevelChildren(spTreeXml) {
  // spTree 바로 아래의 sp / pic / graphicFrame / grpSp / cxnSp 를 순서대로 (중첩 그룹은 통째)
  const out = [];
  const re = /<p:(sp|pic|graphicFrame|grpSp|cxnSp)\b/g;
  let m;
  let pos = 0;
  while ((m = re.exec(spTreeXml))) {
    if (m.index < pos) continue;
    const tag = m[1];
    const start = m.index;
    // 같은 태그의 중첩을 고려해 닫는 위치 찾기
    let depth = 0, i = start;
    const open = new RegExp(`<p:${tag}\\b[^>]*?(/>|>)`, 'g'), closeTag = `</p:${tag}>`;
    open.lastIndex = start;
    while (true) {
      open.lastIndex = i;
      const o = open.exec(spTreeXml);
      const c = spTreeXml.indexOf(closeTag, i);
      if (o && o.index < c) { if (o[1] === '>') depth += 1; i = o.index + o[0].length; }
      else { depth -= 1; i = c + closeTag.length; if (depth === 0) break; }
    }
    out.push({ tag, xml: spTreeXml.slice(start, i) });
    pos = i;
  }
  return out;
}
function spTreeOf(slideXml) {
  const m = slideXml.match(/<p:spTree>([\s\S]*)<\/p:spTree>/);
  if (!m) throw new Error('spTree 없음');
  // 앞의 nvGrpSpPr / grpSpPr 제거
  return m[1].replace(/^[\s\S]*?<\/p:grpSpPr>/, '');
}
const isSldNum = (xml) => /<p:ph [^>]*type="(sldNum|dt|ftr)"/.test(xml);

async function main() {
  const deck = await JSZip.loadAsync(fs.readFileSync(DECK));
  const poster = await JSZip.loadAsync(fs.readFileSync(TEMPLATE));
  const rd = (zip, p) => zip.file(p).async('string');

  // 슬라이드 순서
  const presXml = await rd(deck, 'ppt/presentation.xml');
  const presRels = await rd(deck, 'ppt/_rels/presentation.xml.rels');
  const relMap = {};
  for (const m of presRels.matchAll(/Id="(rId\d+)"[^>]*Target="([^"]+)"/g)) relMap[m[1]] = m[2];
  const slidePaths = [...presXml.matchAll(/<p:sldId [^>]*r:id="(rId\d+)"/g)].map((m) => 'ppt/' + relMap[m[1]]);
  if (slidePaths.length !== 12) throw new Error(`발표자료가 12장이어야 한다 (지금 ${slidePaths.length})`);

  // 포스터 칸 위치 (4행×3열) — 그룹 안의 흰 사각형 절대 좌표
  let posterXml = await rd(poster, 'ppt/slides/slide1.xml');
  const boxes = [];
  for (const g of posterXml.matchAll(/<p:grpSp>[\s\S]*?<\/p:grpSp>/g)) {
    const gx = g[0];
    const names = [...gx.matchAll(/<p:cNvPr id="\d+" name="([^"]+)"/g)].map((m) => m[1]);
    if (!/^그룹 (9|10|14|18)$/.test(names[0])) continue;
    const xf = gx.match(/<p:grpSpPr>[\s\S]*?<a:off x="(-?\d+)" y="(-?\d+)"\/><a:ext cx="(\d+)" cy="(\d+)"\/><a:chOff x="(-?\d+)" y="(-?\d+)"\/><a:chExt cx="(\d+)" cy="(\d+)"\/>/);
    const [ox, oy, ex, ey, cox, coy, cex, cey] = xf.slice(1).map(Number);
    const sx = ex / cex, sy = ey / cey;
    for (const sp of gx.matchAll(/<p:sp>[\s\S]*?<\/p:sp>/g)) {
      const o = sp[0].match(/<a:off x="(-?\d+)" y="(-?\d+)"\/><a:ext cx="(\d+)" cy="(\d+)"\/>/);
      if (!o) continue;
      boxes.push({ x: Math.round(ox + (Number(o[1]) - cox) * sx), y: Math.round(oy + (Number(o[2]) - coy) * sy), w: Math.round(Number(o[3]) * sx), h: Math.round(Number(o[4]) * sy) });
    }
  }
  boxes.sort((a, b) => (a.y - b.y) || (a.x - b.x));
  if (boxes.length !== 12) throw new Error(`포스터 칸 12개여야 한다 (지금 ${boxes.length})`);

  // 포스터 slide1 rels / 미디어
  const relsPath = 'ppt/slides/_rels/slide1.xml.rels';
  let posterRels = await rd(poster, relsPath);
  let nextRid = Math.max(...[...posterRels.matchAll(/Id="rId(\d+)"/g)].map((m) => Number(m[1]))) + 1;
  let nextId = Math.max(...[...posterXml.matchAll(/<p:cNvPr id="(\d+)"/g)].map((m) => Number(m[1]))) + 1;
  let mediaN = 0;
  const addImage = async (zipPath) => {
    const data = await deck.file(zipPath).async('nodebuffer');
    const ext = zipPath.split('.').pop().toLowerCase();
    mediaN += 1;
    const name = `ppt/media/poster-slide-${mediaN}.${ext}`;
    poster.file(name, data);
    const rid = `rId${nextRid++}`;
    posterRels = posterRels.replace('</Relationships>', `<Relationship Id="${rid}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="../media/${name.split('/').pop()}"/></Relationships>`);
    return rid;
  };

  const groups = [];
  for (let i = 0; i < 12; i++) {
    const sp = slidePaths[i];
    const slideXml = await rd(deck, sp);
    const relsXml = await rd(deck, sp.replace('slides/', 'slides/_rels/') + '.rels');
    const rels = {};
    for (const m of relsXml.matchAll(/Id="(rId\d+)"[^>]*Target="([^"]+)"/g)) rels[m[1]] = m[2];
    const layoutRel = Object.values(rels).find((t) => t.includes('slideLayouts/'));
    const layoutPath = 'ppt/slideLayouts/' + layoutRel.split('/').pop();
    const layoutXml = await rd(deck, layoutPath);
    const layoutRelsXml = await rd(deck, layoutPath.replace('slideLayouts/', 'slideLayouts/_rels/') + '.rels');
    const lrels = {};
    for (const m of layoutRelsXml.matchAll(/Id="(rId\d+)"[^>]*Target="([^"]+)"/g)) lrels[m[1]] = m[2];

    // 레이아웃 도형(틀: 선·로고·띠) 먼저, 그 위에 슬라이드 도형
    const parts = [];
    for (const [xml, rmap, base] of [[layoutXml, lrels, 'ppt/slideLayouts/'], [slideXml, rels, 'ppt/slides/']]) {
      for (const ch of topLevelChildren(spTreeOf(xml))) {
        if (isSldNum(ch.xml)) continue;
        let x = ch.xml;
        // 그림 관계 id 재배정
        const embeds = [...new Set([...x.matchAll(/r:embed="(rId\d+)"/g)].map((m) => m[1]))];
        for (const rid of embeds) {
          const target = rmap[rid];
          if (!target) throw new Error(`${sp}: ${rid} 관계 없음`);
          const zipPath = (base + target).replace(/[^/]+\/\.\.\//g, '');
          const nrid = await addImage(zipPath);
          x = x.replace(new RegExp(`r:embed="${rid}"`, 'g'), `r:embed="${nrid}"`);
        }
        x = x.replace(/<a:extLst>[\s\S]*?<\/a:extLst>/g, '');          // a16 등 확장 제거
        x = x.replace(/<p:ph [^>]*\/>/g, '');                            // 자리표시자 상속 끊기 (xfrm·글자속성은 명시돼 있음)
        x = x.replace(/<p:nvPr><\/p:nvPr>/g, '<p:nvPr/>');
        x = fixSchemeColors(x);
        x = fixFonts(x);
        x = x.replace(/<p:cNvPr id="\d+"/g, () => `<p:cNvPr id="${nextId++}"`);
        parts.push(x);
      }
    }
    // 표(graphicFrame)는 PowerPoint 에서 그룹에 못 넣는다 → 그룹 대신 도형마다 좌표를 칸 위치만큼 옮긴다
    const b = boxes[i];
    for (const x0 of parts) {
      let done = false;
      const x = x0.replace(/<a:off x="(-?\d+)" y="(-?\d+)"\/>/, (m, ox, oy) => {
        done = true;
        return `<a:off x="${Number(ox) + b.x}" y="${Number(oy) + b.y}"/>`;
      });
      if (!done) throw new Error(`슬라이드 ${i + 1}: xfrm 없는 도형`);
      groups.push(x);
    }
  }

  // 팀명·학교·팀원
  posterXml = posterXml.replace('<a:t>팀 명</a:t>', `<a:t>${xmlEsc(TEAM.name)}</a:t>`);
  posterXml = posterXml.replace(/<a:t>\s*학교<\/a:t>/, `<a:t> ${xmlEsc(SCHOOL)}</a:t>`);
  posterXml = posterXml.replace(/(<p:sp>(?:(?!<\/p:sp>)[\s\S])*?name="TextBox 25"[\s\S]*?<p:txBody>)([\s\S]*?)(<\/p:txBody>)/, (m, head, body, tail) => {
    const firstRun = body.match(/<a:rPr[^>]*>[\s\S]*?<\/a:rPr>/)[0];
    const pPr = (body.match(/<a:pPr>[\s\S]*?<\/a:pPr>/) || [''])[0];
    const run = (t) => `<a:r>${firstRun}<a:t>${xmlEsc(t)}</a:t></a:r>`;
    return head + `<a:p>${pPr}${run('팀  원 : ' + TEAM.members)}</a:p>` + tail;
  });
  posterXml = posterXml.replace('</p:spTree>', groups.join('') + '</p:spTree>');

  poster.file('ppt/slides/slide1.xml', posterXml);
  poster.file(relsPath, posterRels);
  let ct = await rd(poster, '[Content_Types].xml');
  for (const [ext, type] of [['png', 'image/png'], ['jpeg', 'image/jpeg'], ['jpg', 'image/jpeg']]) {
    if (!ct.includes(`Extension="${ext}"`)) ct = ct.replace('<Default Extension="xml"', `<Default Extension="${ext}" ContentType="${type}"/><Default Extension="xml"`);
  }
  poster.file('[Content_Types].xml', ct);
  fs.writeFileSync(OUT, await poster.generateAsync({ type: 'nodebuffer', compression: 'DEFLATE' }));
  console.log(`${OUT} — 12칸 채움, 그림 ${mediaN}개`);
}
main().catch((e) => { console.error(e); process.exit(1); });
