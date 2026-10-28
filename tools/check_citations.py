#!/usr/bin/env python3
"""보고서 원고의 [근거: …] 표시를 저장소와 대조한다.

    python3 tools/check_citations.py docs/전달용/보고서_분담/초안/IMU.md [--show]

검사: ① 파일이 있나 ② 행 번호가 파일 길이 안인가 ③ 커밋이 있나.
--show 면 근거마다 '원고 문장 → 그 행의 실제 내용' 을 나란히 찍는다(의미 대조는 사람이 한다).
형식: 경로:행 · 경로:행-행 · 경로:행,행 · 커밋 abc1234 · 여러 개는 ; 또는 , 로 구분.
"""
import os
import re
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
CITE = re.compile(r'\[근거:\s*([^\]]+)\]')
LOC = re.compile(r'([\w./가-힣Ⅲ_\-]+\.[A-Za-z0-9]+):(\d+)(?:\s*[-~–]\s*(\d+))?((?:\s*,\s*\d+(?:\s*[-~–]\s*\d+)?)*)')
COMMIT = re.compile(r'(?:커밋\s*)?\b([0-9a-f]{7,40})\b')


def commit_ok(h):
    return subprocess.run(['git', 'cat-file', '-e', h + '^{commit}'], cwd=ROOT,
                          capture_output=True).returncode == 0


def main():
    path = sys.argv[1]
    show = '--show' in sys.argv
    lines = open(path, encoding='utf-8').read().split('\n')
    n_ok = n_bad = 0
    cache = {}
    for ln, text in enumerate(lines, 1):
        for m in CITE.finditer(text):
            body = m.group(1)
            for loc in LOC.finditer(body):
                f, a, b, more = loc.group(1), int(loc.group(2)), loc.group(3), loc.group(4)
                full = os.path.join(ROOT, f)
                if f not in cache:
                    cache[f] = open(full, encoding='utf-8', errors='replace').read().split('\n') if os.path.isfile(full) else None
                src = cache[f]
                ranges = [(a, int(b) if b else a)]
                for extra in re.findall(r'(\d+)(?:\s*[-~–]\s*(\d+))?', more or ''):
                    ranges.append((int(extra[0]), int(extra[1]) if extra[1] else int(extra[0])))
                if src is None:
                    n_bad += 1
                    print(f'❌ {path}:{ln}  파일 없음: {f}')
                    continue
                for lo, hi in ranges:
                    if not (1 <= lo <= hi <= len(src)):
                        n_bad += 1
                        print(f'❌ {path}:{ln}  행 범위 밖: {f}:{lo}-{hi} (파일 {len(src)}줄)')
                        continue
                    n_ok += 1
                    if show:
                        claim = CITE.sub('', text).strip()[:110]
                        print(f'— 원고 {ln}: {claim}')
                        for k in range(lo, min(hi, lo + 2) + 1):
                            print(f'    {f}:{k}: {src[k - 1].strip()[:150]}')
            rest = LOC.sub('', body)
            for c in COMMIT.finditer(rest):
                if commit_ok(c.group(1)):
                    n_ok += 1
                else:
                    n_bad += 1
                    print(f'❌ {path}:{ln}  커밋 없음: {c.group(1)}')
    print(f'\n근거 {n_ok + n_bad}개 — 정상 {n_ok}, 문제 {n_bad}')
    sys.exit(1 if n_bad else 0)


if __name__ == '__main__':
    main()
