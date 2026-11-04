"""표 격자(tblGrid) 정리 — 셀 수와 열 수를 맞추고 각 셀에 폭(tcW) 지정. 한글(HWP) 변환 때 표 사라짐 방지.
python3 fixgrid.py in.docx out.docx ref10.docx"""
import html, re, shutil, sys, zipfile, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dx
src, out, ref = sys.argv[1:4]
T = lambda x: html.unescape(''.join(re.findall(r'<w:t[^>]*>([^<]*)</w:t>', x)))
rd = zipfile.ZipFile(ref).read("word/document.xml").decode()
refw = []
for t in re.finditer(r"<w:tbl>.*?</w:tbl>", rd, re.S):
    x = t.group(0)
    rows = re.findall(r"<w:tr[ >].*?</w:tr>", x, re.S)
    n = len(re.findall(r"<w:tc>", rows[0]))
    w = [int(v) for v in re.findall(r'<w:gridCol w:w="(\d+)"', x)][:n]
    refw.append((T(rows[0])[:8], T(rows[1])[:6] if len(rows) > 1 else "", w))
shutil.copy(src, out)
D = dx.Doc(out)
done = []
def fix(m):
    x = m.group(0)
    rows = re.findall(r"<w:tr[ >].*?</w:tr>", x, re.S)
    n = max(len(re.findall(r"<w:tc>", r)) for r in rows)
    tblw = int(re.search(r'<w:tblW w:w="(\d+)"', x).group(1)) if re.search(r'<w:tblW w:w="(\d+)"', x) else 9360
    h0 = T(rows[0])[:8]; h1 = T(rows[1])[:6] if len(rows) > 1 else ""
    cand = [w for a, b, w in refw if a == h0 and len(w) == n]
    if len(cand) > 1:
        cand = [w for a, b, w in refw if a == h0 and b == h1 and len(w) == n] or cand
    if cand and not (h0 in ('값', '배') or h0.startswith(('항목값', '항목배'))):
        ws = cand[0]
    elif h0.startswith("상황"):
        ws = [2808, 3931, 2621]
    elif h0 in ("값", "배") or h0.startswith("항목값") or h0.startswith("항목배"):
        ws = [3360, 6000]
    elif h0.startswith("부하"):
        ws = [3600, 1800, 3960]
    elif h0.startswith("품목"):
        ws = [480, 3810, 900, 1905, 2265]
    else:
        ws = [tblw // n] * n; ws[-1] = tblw - sum(ws[:-1])
    s = sum(ws); ws = [round(w * tblw / s) for w in ws]; ws[-1] = tblw - sum(ws[:-1])
    grid = "<w:tblGrid>" + "".join(f'<w:gridCol w:w="{w}"/>' for w in ws) + "</w:tblGrid>"
    x = re.sub(r"<w:tblGrid>.*?</w:tblGrid>(?:\s*</w:tblGridChange>\s*</w:tblGrid>)*", grid, x, count=1, flags=re.S)
    def row(rm):
        r = rm.group(0); k = [0]
        def cell(cm):
            c = cm.group(0); i = k[0]; k[0] += 1
            w = ws[i] if i < len(ws) else ws[-1]
            c = re.sub(r"<w:tcW [^>]*/>", "", c)
            if "<w:tcPr>" in c:
                return c.replace("<w:tcPr>", f'<w:tcPr><w:tcW w:w="{w}" w:type="dxa"/>', 1)
            return c.replace("<w:tc>", f'<w:tc><w:tcPr><w:tcW w:w="{w}" w:type="dxa"/></w:tcPr>', 1)
        return re.sub(r"<w:tc>.*?</w:tc>", cell, r, flags=re.S)
    x = re.sub(r"<w:tr[ >].*?</w:tr>", row, x, flags=re.S)
    done.append((h0, n, ws))
    return x
D.doc = re.sub(r"<w:tbl>.*?</w:tbl>", fix, D.doc, flags=re.S)
D.save()
for h, n, ws in done: print(h, n, ws)
