# 공통 발표 대본 md(`[A]`·`[B]`·`[/]` 표시)에서 배 하나 전용 md 를 만든다.
# 사용: python3 tools/split_boat_script.py <공통.md> <A|B> <출력.md>
import re
import sys

src, boat, out = sys.argv[1], sys.argv[2], sys.argv[3]
other = "B" if boat == "A" else "A"
MARK = re.compile(r"`\[(A|B|/)\]`")


def resolve(t):
    parts = MARK.split(t)
    res, keep = parts[0], True
    for i in range(1, len(parts), 2):
        m, seg = parts[i], parts[i + 1]
        keep = (m != other)
        if keep:
            res += seg.strip() if m == boat else seg
    return re.sub(r"[ ]{2,}", " ", res)


def only_other(t):
    r = t.lstrip("|> ").strip()
    return r.startswith(f"`[{other}]`") and f"`[{boat}]`" not in t


lines = open(src, encoding="utf-8").read().split("\n")
o = []
for l in lines:
    if l.startswith("# "):
        o.append(f"# 설계심사 발표 대본 — S.S.F {boat} (11-05)")
        continue
    if l.startswith("기준:"):
        rep = "A 최종본" if boat == "A" else "B 15차"
        o.append(f"기준: 발표자료 S.S.F {boat} 12장, **{rep}** 보고서. 숫자는 전부 발표자료·보고서에 있는 값이다.")
        continue
    if l.startswith("- A와 B가 다른 문장은"):
        o.append(f"- S.S.F {boat} 전용 대본이다. 공통 원본은 발표_대본_20261105.md 이고, 이 파일은 tools/split_boat_script.py 로 만든다.")
        continue
    if only_other(l):
        continue
    if l.startswith("|"):
        cells = l.strip().strip("|").split("|")
        o.append("| " + " | ".join(resolve(c).strip() for c in cells) + " |")
    else:
        o.append(resolve(l).rstrip() if MARK.search(l) else l)
open(out, "w", encoding="utf-8").write("\n".join(o))
print("ok", out)
