"""설명서 md → 단독 HTML (docs/절차/*_설명서.html 과 같은 스타일). 사용: python3 tools/md_manual_to_html.py <입력.md> <출력.html>"""
import html
import re
import sys

import markdown

src, out = sys.argv[1], sys.argv[2]
text = open(src, encoding="utf-8").read()
title = re.search(r"^# (.+)$", text, re.M).group(1).strip()
body = markdown.markdown(text, extensions=["tables", "fenced_code", "toc"])
body = body.replace("\\|", "|")   # 표 안 코드의 \| (md 표 구분자 이스케이프) 를 | 로
# 경고 표시가 있는 문단·목록 항목 강조
body = re.sub(r"<(p|li)>(🚨)", r'<\1 class="warn">\2', body)
heads = re.findall(r'<h2 id="([^"]+)">(.+?)</h2>', body)
toc = "".join(f'<a href="#{i}">{re.sub("<[^>]+>", "", t)}</a>' for i, t in heads)
CSS = """
:root{--ink:#1f2328;--mut:#59636e;--bg:#fff;--card:#f6f8fa;--line:#d0d7de;--acc:#1f3864;--warn:#b8860b;--bad:#c00000;--badbg:#fdf0f0}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--ink:#e6edf3;--mut:#9aa4ae;--bg:#0d1117;--card:#161b22;--line:#30363d;--acc:#9ec5ff;--bad:#ff7b72;--badbg:#2a1515;color-scheme:dark}}
:root[data-theme=dark]{--ink:#e6edf3;--mut:#9aa4ae;--bg:#0d1117;--card:#161b22;--line:#30363d;--acc:#9ec5ff;--bad:#ff7b72;--badbg:#2a1515;color-scheme:dark}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.6 "Apple SD Gothic Neo","Malgun Gothic",system-ui,sans-serif}
main{max-width:1100px;margin:0 auto;padding:24px max(16px,env(safe-area-inset-right,0px)) 60px max(16px,env(safe-area-inset-left,0px))}
h1{font-size:27px;color:var(--acc);margin:0 0 8px;padding-bottom:10px;border-bottom:3px solid var(--acc)}
h2{font-size:20px;color:var(--acc);margin:36px 0 10px;padding-top:12px;border-top:1px solid var(--line)}
h3{font-size:16px;margin:22px 0 8px}
table{border-collapse:collapse;width:100%;margin:10px 0;font-size:14px;display:block;overflow-x:auto}
th,td{border:1px solid var(--line);padding:6px 9px;text-align:left;vertical-align:top}th{background:var(--card);white-space:nowrap}
pre{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:12px;overflow-x:auto;font-size:13px;line-height:1.45}
code{font-family:ui-monospace,Menlo,monospace;font-size:.92em;background:var(--card);padding:1px 4px;border-radius:4px}pre code{background:none;padding:0}
blockquote{margin:0 0 16px;padding:8px 14px;border-left:4px solid var(--acc);background:var(--card);color:var(--mut)}
.warn{background:var(--badbg);border-left:4px solid var(--bad);padding:4px 10px;border-radius:4px}
.toc{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px 16px;font-size:14px;margin:14px 0}
.toc a{color:var(--acc);text-decoration:none;margin-right:14px;display:inline-block}
hr{border:0;border-top:1px solid var(--line);margin:20px 0}li{margin:3px 0}
"""
body = body.replace("</h1>", '</h1><div class="toc">' + toc + "</div>", 1)
page = (f'<!doctype html><html lang="ko"><head><meta charset="utf-8">'
        f'<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">'
        f'<title>{html.escape(title)}</title><style>{CSS}</style></head><body><main>'
        f'{body}</main></body></html>')
open(out, "w", encoding="utf-8").write(page)
print(out)
