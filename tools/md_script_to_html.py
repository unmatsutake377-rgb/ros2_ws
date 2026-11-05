# 발표 대본 md -> HTML (A/B 전환 버튼, 연습 타이머). 사용: python3 tools/md_script_to_html.py <대본.md> <출력.html>
import re, sys, html

src, out = sys.argv[1], sys.argv[2]
lines = open(src, encoding="utf-8").read().split("\n")


def inline(t):
    t = html.escape(t, quote=False)
    t = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", t)
    t = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", lambda m: '<span class="ref">' + m.group(1) + "</span>", t)
    # boat markers: `[A]` text `[B]` text
    parts = re.split(r"`\[(A|B|/)\]`", t)
    if len(parts) > 1:
        res = parts[0]
        for i in range(1, len(parts), 2):
            b, seg = parts[i], parts[i + 1]
            res += seg if b == "/" else f'<span class="boat b{b.lower()}"><span class="tag">{b}</span>{seg.strip()}</span>'
        t = res
    t = re.sub(r"`([^`]+)`", r"<code>\1</code>", t)
    return t


def boatclass(raw):
    r = raw.strip().lstrip("|").strip()
    a, b = "`[A]`" in raw, "`[B]`" in raw
    if r.startswith("`[A]`") and not b:
        return " only-a"
    if r.startswith("`[B]`") and not a:
        return " only-b"
    return ""


o = []
i = 0
sec_open = False
slide_open = False
while i < len(lines):
    l = lines[i]
    if l.startswith("# "):
        o.append(f'<header class="top"><h1>{inline(l[2:])}</h1></header>')
    elif l.startswith("## "):
        if slide_open:
            o.append("</article>"); slide_open = False
        if sec_open:
            o.append("</section>")
        title = l[3:]
        sid = "s" + str(len(o))
        o.append(f'<section id="{sid}"><h2>{inline(title)}</h2>')
        sec_open = True
    elif l.startswith("### "):
        if slide_open:
            o.append("</article>")
        t = l[4:]
        m = re.match(r"(\d+)장 (.+?) \((\d+) s\)$", t)
        if m:
            o.append(f'<article class="slide"><div class="shead"><span class="num">{m.group(1)}</span>'
                     f'<h3>{inline(m.group(2))}</h3><span class="sec">{m.group(3)} s</span></div>')
        else:
            o.append(f'<article class="part"><h3>{inline(t)}</h3>')
        slide_open = True
    elif l.startswith(">"):
        para = []
        while i < len(lines) and lines[i].startswith(">"):
            para.append(lines[i][1:].strip()); i += 1
        o.append('<div class="say">')
        for p in para:
            if p:
                o.append(f'<p class="line{boatclass(p)}">{inline(p)}</p>')
        o.append("</div>")
        continue
    elif l.startswith("|"):
        rows = []
        while i < len(lines) and lines[i].startswith("|"):
            rows.append(lines[i]); i += 1
        cells = [[c.strip() for c in r.strip().strip("|").split("|")] for r in rows]
        head, body = cells[0], [r for r in cells[2:]]
        o.append('<div class="tw"><table><thead><tr>' + "".join(f"<th>{inline(c)}</th>" for c in head) + "</tr></thead><tbody>")
        for r, raw in zip(body, rows[2:]):
            o.append(f'<tr class="{boatclass(raw).strip()}">' + "".join(f"<td>{inline(c)}</td>" for c in r) + "</tr>")
        o.append("</tbody></table></div>")
        continue
    elif l.startswith("- "):
        o.append("<ul>")
        while i < len(lines) and lines[i].startswith("- "):
            o.append(f"<li>{inline(lines[i][2:])}</li>"); i += 1
        o.append("</ul>")
        continue
    elif l.strip() == "---" or not l.strip():
        pass
    else:
        o.append(f"<p>{inline(l)}</p>")
    i += 1
if slide_open:
    o.append("</article>")
if sec_open:
    o.append("</section>")

body = "\n".join(o)
has_boat = bool(re.search(r"`\[(A|B)\]`", "\n".join(lines)))
BOATBTN = '<button data-b="all" class="on">A·B 모두</button><button data-b="a">A만</button><button data-b="b">B만</button>\n<span class="sep"></span>\n'

page = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>SSF 설계심사 발표 대본</title>
<style>
:root{--bg:#f6f7f9;--card:#fff;--fg:#1b2230;--mut:#5d6778;--line:#dfe3ea;--accent:#1f3a68;--a:#0f6e8c;--b:#9a4a14;--abg:#e6f3f7;--bbg:#fbefe5;--say:#f2f5fa;color-scheme:light}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#11151c;--card:#1a2029;--fg:#e6e9ef;--mut:#9aa4b5;--line:#2c3542;--accent:#8fb2ec;--a:#5cc3e0;--b:#f0a66e;--abg:#12303a;--bbg:#3a2616;--say:#1f2733;color-scheme:dark}}
:root[data-theme="dark"]{--bg:#11151c;--card:#1a2029;--fg:#e6e9ef;--mut:#9aa4b5;--line:#2c3542;--accent:#8fb2ec;--a:#5cc3e0;--b:#f0a66e;--abg:#12303a;--bbg:#3a2616;--say:#1f2733;color-scheme:dark}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:16px/1.7 -apple-system,"Apple SD Gothic Neo","Malgun Gothic","Noto Sans KR",sans-serif;word-break:keep-all}
.wrap{max-width:880px;margin:0 auto;padding:0 max(16px,env(safe-area-inset-left,0px)) 80px max(16px,env(safe-area-inset-right,0px))}
.bar{position:sticky;top:0;z-index:5;background:var(--bg);border-bottom:1px solid var(--line);padding:10px max(16px,env(safe-area-inset-left,0px));display:flex;flex-wrap:wrap;gap:8px;align-items:center;justify-content:center}
.bar button{font:inherit;font-size:14px;padding:6px 14px;border-radius:999px;border:1px solid var(--line);background:var(--card);color:var(--fg);cursor:pointer}
.bar button.on{background:var(--accent);border-color:var(--accent);color:var(--bg)}
.bar .timer{font-variant-numeric:tabular-nums;font-weight:700;min-width:58px;text-align:center}
.bar .sep{width:1px;height:22px;background:var(--line)}
header.top h1{font-size:1.5rem;line-height:1.35;margin:28px 0 8px;text-wrap:balance;color:var(--accent)}
h2{font-size:1.25rem;margin:40px 0 14px;padding-bottom:6px;border-bottom:2px solid var(--accent)}
h3{font-size:1.05rem;margin:0}
p{margin:8px 0}
.ref{color:var(--accent)}
code{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:.88em;background:var(--say);padding:1px 5px;border-radius:4px}
ul{padding-left:1.2em;margin:8px 0}
.slide,.part{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px 18px;margin:14px 0}
.shead{display:flex;align-items:center;gap:10px;margin-bottom:8px}
.shead .num{flex:none;width:30px;height:30px;border-radius:50%;background:var(--accent);color:var(--bg);display:grid;place-items:center;font-weight:700;font-size:14px}
.shead h3{flex:1}
.shead .sec{flex:none;font-size:13px;color:var(--mut);border:1px solid var(--line);border-radius:999px;padding:1px 10px}
.say{background:var(--say);border-radius:8px;padding:6px 14px}
.say p{font-size:1.06rem;line-height:1.85}
.boat .tag{display:inline-block;font-size:11px;font-weight:700;line-height:1;padding:3px 6px;border-radius:4px;margin:0 6px 0 2px;vertical-align:2px;color:#fff}
.ba .tag{background:var(--a)} .bb .tag{background:var(--b)}
.ba{background:var(--abg);border-radius:4px} .bb{background:var(--bbg);border-radius:4px}
body.show-a .bb, body.show-a .only-b{display:none}
body.show-b .ba, body.show-b .only-a{display:none}
body.show-a .ba, body.show-b .bb{background:none}
body.show-a .tag, body.show-b .tag{display:none}
.tw{overflow-x:auto;margin:10px 0}
table{border-collapse:collapse;width:100%;font-size:14.5px;background:var(--card)}
th,td{border:1px solid var(--line);padding:7px 9px;text-align:left;vertical-align:top}
th{background:var(--say);font-weight:700;white-space:nowrap}
td:nth-child(2){white-space:nowrap}
@media (max-width:600px){body{font-size:15px}.say p{font-size:1rem}td:nth-child(2){white-space:normal}}
</style></head>
<body class="show-all">
<div class="bar">
""" + (BOATBTN if has_boat else "") + """<span class="timer" id="t">0:00</span><button id="go">시작</button><button id="rs">초기화</button>
</div>
<main class="wrap">
""" + body + """
</main>
<script>
(function(){
var bs=document.querySelectorAll('[data-b]');
function set(b){document.body.className='show-'+b;bs.forEach(function(x){x.classList.toggle('on',x.dataset.b===b)});try{localStorage.setItem('ssf-boat',b)}catch(e){}}
bs.forEach(function(x){x.addEventListener('click',function(){set(x.dataset.b)})});
try{var s=localStorage.getItem('ssf-boat');if(s)set(s)}catch(e){}
var t0=0,acc=0,run=false,id=null,el=document.getElementById('t'),go=document.getElementById('go');
function fmt(ms){var s=Math.floor(ms/1000);return Math.floor(s/60)+':'+String(s%60).padStart(2,'0')}
function tick(){var ms=acc+(run?Date.now()-t0:0);el.textContent=fmt(ms);el.style.color=ms>300000?'#d23':''}
go.addEventListener('click',function(){if(run){acc+=Date.now()-t0;run=false;clearInterval(id);go.textContent='계속'}else{t0=Date.now();run=true;id=setInterval(tick,250);go.textContent='멈춤'}tick()});
document.getElementById('rs').addEventListener('click',function(){run=false;clearInterval(id);acc=0;go.textContent='시작';tick()});
})();
</script>
</body></html>
"""
open(out, "w", encoding="utf-8").write(page)
print("ok", len(page))
