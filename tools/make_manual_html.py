#!/usr/bin/env python3
"""마크다운 설명서 → 그림을 base64 로 내장한 단일 HTML.
사용: python3 tools/make_manual_html.py docs/절차/허브_카메라_전원_설명서.md
출력: 같은 이름 .html (옆에 저장). 상대경로 이미지(.png/.jpg)를 md 위치 기준으로 찾아 내장한다."""
import base64
import os
import re
import sys

import markdown

src = sys.argv[1]
base = os.path.dirname(os.path.abspath(src))
md = open(src, encoding="utf-8").read()


def embed(m):
    alt, rel = m.group(1), m.group(2)
    p = os.path.join(base, rel)
    if not os.path.exists(p):
        return m.group(0)
    ext = os.path.splitext(p)[1].lstrip(".").lower().replace("jpg", "jpeg")
    data = base64.b64encode(open(p, "rb").read()).decode()
    return f'![{alt}](data:image/{ext};base64,{data})'


md = re.sub(r'!\[([^\]]*)\]\(([^)]+\.(?:png|jpg|jpeg))\)', embed, md)
body = markdown.markdown(md, extensions=["tables", "fenced_code"])
title = re.search(r'^#\s+(.+)$', md, flags=re.M).group(1)
html = f"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{title}</title>
<style>
body{{font-family:"Apple SD Gothic Neo","Malgun Gothic",sans-serif;max-width:1100px;margin:24px auto;padding:0 16px;line-height:1.55;color:#222}}
h1{{font-size:1.5em;border-bottom:2px solid #333;padding-bottom:6px}} h2{{font-size:1.2em;margin-top:1.6em;border-left:5px solid #b8860b;padding-left:8px}}
table{{border-collapse:collapse;width:100%;font-size:.92em;margin:10px 0}} th,td{{border:1px solid #999;padding:5px 8px;vertical-align:top}} th{{background:#eee}}
img{{max-width:100%;height:auto;border:1px solid #ccc;margin:8px 0}} blockquote{{color:#555;border-left:4px solid #ccc;margin:0;padding:4px 12px}}
code{{background:#f3f3f3;padding:1px 4px}} ul{{padding-left:1.3em}}
@media print{{body{{max-width:none}} h2{{page-break-after:avoid}} img{{page-break-inside:avoid}}}}
</style></head><body>{body}</body></html>"""
out = os.path.splitext(src)[0] + ".html"
open(out, "w", encoding="utf-8").write(html)
print("wrote", out, len(html) // 1024, "KB")
