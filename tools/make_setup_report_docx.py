# -*- coding: utf-8 -*-
"""SSF 주간 보고서 양식(활동 일지 양식 수정본.docx)의 스타일·표 XML 을 그대로 재사용해 '세팅 작업 보고서' docx 를 만든다."""
import zipfile, re, html, sys, io, shutil, os
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "전달용", "양식", "SSF_주간보고서_양식_원본.docx")
OUT = sys.argv[1]
z = zipfile.ZipFile(SRC); x = z.read("word/document.xml").decode("utf8")
body = re.search(r"<w:body>(.*)</w:body>", x, re.S).group(1)
blocks = re.findall(r"<w:tbl>.*?</w:tbl>|<w:p(?: [^>]*)?>.*?</w:p>|<w:p(?: [^>]*)?/>|<w:sectPr.*?</w:sectPr>", body, re.S)
P_TITLE, P_SUB, P_H1, P_NORM, P_BULLET, P_EMPTY = blocks[25], blocks[26], blocks[27], blocks[29], blocks[3], blocks[28]
TBL = [b for b in blocks if b.startswith("<w:tbl>")][0]
SECT = blocks[-1]
rows = re.findall(r"<w:tr(?: [^>]*)?>.*?</w:tr>", TBL, re.S)
ROW_H, ROW_D = rows[0], rows[1]
esc = lambda s: html.escape(s, quote=False)

def set_text(p_xml, text):
    """문단 XML 의 첫 run 텍스트만 바꾸고 나머지 run 제거. 줄바꿈은 <w:br/>."""
    runs = re.findall(r"<w:r(?: [^>]*)?>.*?</w:r>", p_xml, re.S)
    if not runs:
        return re.sub(r"</w:p>$", f'<w:r><w:t xml:space="preserve">{esc(text)}</w:t></w:r></w:p>', p_xml)
    r0 = runs[0]
    rpr = re.search(r"<w:rPr>.*?</w:rPr>", r0, re.S); rpr = rpr.group(0) if rpr else ""
    parts = text.split("\n")
    inner = "".join(f'<w:t xml:space="preserve">{esc(t)}</w:t>' + ("<w:br/>" if i < len(parts)-1 else "") for i,t in enumerate(parts))
    newr = f"<w:r>{rpr}{inner}</w:r>"
    out = p_xml
    for r in runs: out = out.replace(r, "", 1)
    return re.sub(r"</w:p>$", newr + "</w:p>", out)

def para(tpl, text, bold=None):
    if tpl is P_BULLET:
        tpl = re.sub(r"<w:numPr>.*?</w:numPr>", "", tpl, flags=re.S)
        text = "•  " + text
    p = set_text(tpl, text)
    if bold is True and "<w:b/>" not in p:
        p = p.replace("<w:rPr>", "<w:rPr><w:b/>", 1) if "<w:rPr>" in p else p.replace("<w:r>", "<w:r><w:rPr><w:b/></w:rPr>", 1)
    return p
def pagebreak(): return '<w:p><w:r><w:br w:type="page"/></w:r></w:p>'

def row(cells, header=False, height=None):
    src = ROW_H if header else ROW_D
    tcs = re.findall(r"<w:tc>.*?</w:tc>", src, re.S)
    out_tcs = []
    for tc, text in zip(tcs, cells):
        tcpr = re.search(r"<w:tcPr>.*?</w:tcPr>", tc, re.S).group(0).replace('<w:vMerge w:val="restart"/>', "").replace("<w:vMerge/>", "")
        p0 = re.search(r"<w:p(?: [^>]*)?>.*?</w:p>", tc, re.S).group(0)
        paras = "".join(set_text(p0, line) for line in (text.split("\n\n") if text else [""]))
        out_tcs.append(f"<w:tc>{tcpr}{paras}</w:tc>")
    trpr = re.search(r"<w:trPr>.*?</w:trPr>", src, re.S).group(0)
    if height: trpr = re.sub(r'<w:trHeight w:val="\d+"', f'<w:trHeight w:val="{height}"', trpr)
    return f"<w:tr>{trpr}{''.join(out_tcs)}</w:tr>"

def table(hdr, data, widths=None, heights=1200):
    tblpr = re.search(r"<w:tblPr>.*?</w:tblPr>", TBL, re.S).group(0)
    w = widths or [600, 4080, 2340, 2340]
    grid = "<w:tblGrid>" + "".join(f'<w:gridCol w:w="{v}"/>' for v in w) + "</w:tblGrid>"
    rs = [row(hdr, header=True)] + [row(d, height=heights) for d in data]
    t = f"<w:tbl>{tblpr}{grid}{''.join(rs)}</w:tbl>"
    # 셀 폭을 grid 에 맞춤
    tcs = re.findall(r"<w:tcW [^>]*/>", t)
    i = 0
    def repl(m):
        nonlocal i
        v = w[i % len(w)]; i += 1
        return f'<w:tcW w:w="{v}" w:type="dxa"/>'
    return re.sub(r"<w:tcW [^>]*/>", repl, t)

# ───────────── 내용 ─────────────
PREFILL = len(sys.argv) > 2 and sys.argv[2] == "draft"
D = "2026년 __월 __일" if not PREFILL else "2026년 10월 __일 (부품 전부 도착 후 1일)"
B = []
B.append(para(P_TITLE, "세팅 작업 보고서"))
B.append(para(P_SUB, f"일시 : {D}"))
B.append(para(P_H1, "1. 작업 개요"))
for s in ["일시 / 장소 : ____ / ____",
          "참석 : 소프트 ____ · 회로 ____ · 선체 ____",
          "역할 분담 : 회로 — 분기 피그테일·퓨즈홀더·5V 모듈·인젝터 결선 / 소프트 — 노트북 IP·드라이버·토픽·화각 / 선체 — 박스 내 고정·관통부",
          "목표 : 카메라 계통(A)·허브 계통(E) 결선 완료 + OAK 영상 토픽 확인 + 허브에 라이다·IMU·GPS·D455 인식"]:
    B.append(para(P_BULLET, s))
B.append(para(P_H1, "2. 오늘 할 일 (미리 적어두기)"))
todo = ["회로: BF303 퓨즈홀더 2개 조립(2A 인젝터 / 3A 허브, 유성펜 라벨)",
        "회로: EC5 암 피그테일 ⊕ 두 갈래 분기 — 납땜+수축튜브 (OAK_허브_세팅_설명서 §2)",
        "회로: SZWENGAO 5V 모듈 결선 → 출력 4.9~5.1V 확인 → DC 플러그(중심 +) → UH308 (§3)",
        "회로: PSE5502G 인젝터 DC IN 극성 확인 → PoE OUT→OAK Cat5e / LAN→NM-UCL02 (§4)",
        "소프트: 노트북 USB-A① Mega · USB-A② 허브 · USB-C 어댑터, nmcli 169.254.1.10/16, ping 169.254.1.222",
        "소프트: depthai-ros 드라이버, 컬러 토픽 이름 확인 → vision.yaml / ssf_tools.yaml image_topic",
        "소프트: hfov_deg 실측(런북 §5), HSV 재캘리브레이션(§6)",
        "선체: 인젝터·5V 모듈·허브·퓨즈홀더 박스 안 자리 고정, Cat5e 관통부 실란트+드립 루프"]
for s in todo: B.append(para(P_BULLET, s))
B.append(para(P_H1, "3. 다음 작업 계획"))
for s in ["수조: 조향 부호 → reverse → max_diff → base_pwm → half_width 게이트 2m (수조_실측값_반영_프롬프트)",
          "2번배: 동일 회로 결선(Mega·무선스위치 도착), udev 포트 고정(F7), 조종기 중앙·데드밴드 재측정",
          "야외: GPS 측위(F1) · IMU 정북(C5) · LED 한낮 시인성(F2)"]:
    B.append(para(P_BULLET, s))

B.append(para(P_TITLE, "SSF 세팅 활동 보고").replace("<w:pPr>", "<w:pPr><w:pageBreakBefore/>", 1))
B.append(para(P_SUB, D))
hdr = ["일시", "활동사진", "활동내용", "특이사항"]
B.append(para(P_H1, "회로 — 전원 분기 · 퓨즈 · 5V 모듈 · 인젝터"))
B.append(para(P_NORM, "활동내용"))
B.append(table(hdr, [
    ["__/__", "", "퓨즈홀더 2개 조립, 2A/3A 라벨\n\nEC5 피그테일 ⊕ 두 갈래 분기(BF303 2A → 인젝터 / BF303 3A → 5V 모듈)", "퓨즈 딸깍 여부 / 납땜·수축튜브 확인"],
    ["", "", "SZWENGAO 5V 모듈 결선 → OUT 실측 ____ V → DC 플러그 극성 확인 → UH308 급전", "멀티미터 값 기록"],
    ["", "", "PSE5502G 인젝터 DC IN(극성 ____) → PoE OUT→OAK / LAN→어댑터. 인젝터 LED ____ / OAK LED ____", "배터리 전압 ____ V (12.5 이상)"],
]))
B.append(para(P_EMPTY, ""))
B.append(para(P_H1, "소프트웨어 — OAK 연결 · 토픽 · 화각"))
B.append(para(P_NORM, "활동내용"))
B.append(table(hdr, [
    ["__/__", "", "nmcli oak-poe 169.254.1.10/16 적용, ping 169.254.1.222 ____", "VPN·ufw 11490/11491"],
    ["", "", "depthai-ros 설치, 컬러 토픽 이름 ________ , hz ____ , QoS ____", "vision.yaml · ssf_tools.yaml image_topic 두 곳"],
    ["", "", "hfov_deg 실측 ____° (사양 69°, 해상도 ____) / HSV 빨강 ____ 초록 ____ / min_area_px ____", "런북 §5 검산 통과 여부"],
    ["", "", "허브: lsusb 에 LiDAR·IMU·GPS·D455 ____ / 메인 launch /image_angle 발행자 __개", ""],
]))
B.append(para(P_EMPTY, ""))
B.append(para(P_H1, "탑재장비 제원 · 결선 (경쟁팀 보고서 양식에서 가져옴)"))
B.append(para(P_NORM, "회로도 : docs/전달용/배치도/회로도_통합.png (2026-10-26 판) 첨부"))
B.append(table(["장비", "모델", "전원 · 인터페이스", "비고"], [
    ["비전 카메라", "Luxonis OAK-1 PoE FF (69°)", "PoE 802.3af · 이더넷 · IP67", "마스트 위, 4~7.5W"],
    ["PoE 인젝터", "REVOTECH PSE5502G 30W", "DC 12-48V in → 48V af/at · 기가", "박스 안, 비방수, 2A 퓨즈"],
    ["이더넷 어댑터", "NETmate NM-UCL02 (AX88179)", "USB-C 5Gbps → RJ45 1G", "노트북 USB-C 직결"],
    ["USB 허브", "ipTIME UH308 유전원", "5V (SZWENGAO 5V 5A, 3A 퓨즈) · USB 3.0", "LiDAR·IMU·GPS·D455"],
    ["5V 모듈", "SZWENGAO WG-1224S0505", "8~36V in → 5V 고정 5A · IP68", "트리머 없음"],
    ["분기 커넥터", "EC5 암 피그테일 10AWG", "배터리 A ⊕/⊖, 접촉기 앞", "비상정지와 무관하게 유지"],
], widths=[1800, 2800, 2600, 2160], heights=500))
B.append(para(P_EMPTY, ""))
B.append(para(P_H1, "결과 체크리스트"))
for s in ["5V 모듈 OUT 4.9~5.1V · 허브 LED · lsusb 에 4개 장비", "인젝터 LED · OAK LED · ping 169.254.1.222",
          "퓨즈홀더 2개 라벨, 손 닿는 자리", "아두이노 노트북 직결(허브 아님)", "배터리 연결 순서 1·2 → 5·6 으로 켜도 정상"]:
    B.append(para(P_BULLET, "☐ " + s))
B.append(para(P_H1, "문제 · 조치"))
B.append(table(["#", "증상", "원인 / 조치", "상태"], [["1", "", "", ""], ["2", "", "", ""]], widths=[600, 3000, 4000, 1760], heights=500))
B.append(para(P_EMPTY, ""))
B.append(para(P_H1, "소요비용 (경쟁팀 보고서 양식에서 가져옴)"))
B.append(table(["구분", "품목", "금액(원)", "비고"], [
    ["지원금", "OAK-1 PoE ×2 · Mega ×1 · RF 무선스위치 ×1", "1,414,100", "동아리 2차 활동계획서"],
    ["자부담", "인젝터·어댑터·케이블·컨버터·허브·5V 모듈·퓨즈·EC5", "259,670 ~ 274,670", "구매_목록.md §0 (10-26)"],
    ["합계", "12항목", "1,673,770 ~ 1,688,770", ""],
], widths=[1200, 4600, 2000, 1560], heights=500))
B.append(SECT)

new = x[:x.index("<w:body>")+8] + "".join(B) + x[x.index("</w:body>"):]
# zip 재조립 (document.xml 만 교체)
with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as zo:
    for n in z.namelist():
        data = new.encode("utf8") if n == "word/document.xml" else z.read(n)
        zo.writestr(n, data)
print("wrote", OUT)
