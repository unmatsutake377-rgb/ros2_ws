# -*- coding: utf-8 -*-
"""10-31 OAK 시험 '세팅 작업 보고서' docx — 활동사진 칸에 실제 사진을 넣는다.

양식·표 XML 은 tools/make_setup_report_docx.py 와 같은 원본(SSF_주간보고서_양식_원본.docx)을 쓴다.
사진은 docs/전달용/OAK시험_20261031/사진/ 에서 번호 접두어(00_, 05_ …)로 찾는다.
없는 사진 칸에는 "📷 사진 대기 — 파일명" 이 들어가니, 그 이름으로 사진을 넣고 다시 돌리면 채워진다.
수치는 같은 폴더 보고서_인계.md · 허브_카메라_전원_설명서.md §8 실측 그대로 — 새로 지어내지 않는다.

재현: python3 tools/oak/make_oak_report_docx.py
  → docs/전달용/OAK시험_20261031/세팅작업_보고서_OAK_20261031.docx
"""
import glob, html, io, os, re, sys, zipfile
from PIL import Image, ImageOps

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
SRC = os.path.join(ROOT, "docs", "전달용", "양식", "SSF_주간보고서_양식_원본.docx")
DIR = os.path.join(ROOT, "docs", "전달용", "OAK시험_20261031")
PHOTO = os.path.join(DIR, "사진")
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(DIR, "세팅작업_보고서_OAK_20261031.docx")

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
    inner = "".join(f'<w:t xml:space="preserve">{esc(t)}</w:t>' + ("<w:br/>" if i < len(parts) - 1 else "") for i, t in enumerate(parts))
    out = p_xml
    for r in runs: out = out.replace(r, "", 1)
    return re.sub(r"</w:p>$", f"<w:r>{rpr}{inner}</w:r></w:p>", out)


def para(tpl, text):
    if tpl is P_BULLET:
        tpl = re.sub(r"<w:numPr>.*?</w:numPr>", "", tpl, flags=re.S)
        text = "•  " + text
    return set_text(tpl, text)


# ───────────── 사진 ─────────────
MEDIA = []                      # (zip 이름, bytes, rId)
PIC_W_EMU = 3750 * 635          # 활동사진 칸 4080 dxa − 여백
PIC_H_EMU = int(2.3 * 914400)   # 한 장 최대 높이 2.3 in


class Pic:
    """활동사진 칸. prefix 로 사진/ 에서 찾고, 없으면 대기 문구."""
    def __init__(self, prefix, want, caption=""):
        self.prefix, self.want, self.caption = prefix, want, caption


def find_photo(prefix):
    hits = sorted(f for f in glob.glob(os.path.join(PHOTO, prefix + "_*"))
                  if f.lower().endswith((".jpg", ".jpeg", ".png")))
    return hits[0] if hits else None


def drawing(path):
    im = ImageOps.exif_transpose(Image.open(path)).convert("RGB")
    im.thumbnail((1400, 1400))
    buf = io.BytesIO(); im.save(buf, "JPEG", quality=82)
    n = len(MEDIA) + 1; rid = f"rIdAct{n}"; name = f"media/act{n}.jpg"
    MEDIA.append((name, buf.getvalue(), rid))
    w, h = im.size
    s = min(PIC_W_EMU / w, PIC_H_EMU / h); cx, cy = int(w * s), int(h * s)
    pid = 1000 + n
    return (f'<w:r><w:drawing><wp:inline distT="0" distB="0" distL="0" distR="0"><wp:extent cx="{cx}" cy="{cy}"/>'
            f'<wp:docPr id="{pid}" name="act{n}"/><wp:cNvGraphicFramePr><a:graphicFrameLocks noChangeAspect="1"/></wp:cNvGraphicFramePr>'
            '<a:graphic><a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture"><pic:pic>'
            f'<pic:nvPicPr><pic:cNvPr id="{pid}" name="act{n}.jpg"/><pic:cNvPicPr/></pic:nvPicPr>'
            f'<pic:blipFill><a:blip r:embed="{rid}"/><a:stretch><a:fillRect/></a:stretch></pic:blipFill>'
            f'<pic:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{cx}" cy="{cy}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></pic:spPr>'
            '</pic:pic></a:graphicData></a:graphic></wp:inline></w:drawing></w:r>')


MISSING = []


def pic_cell(p0, pic):
    path = find_photo(pic.prefix)
    if not path:
        MISSING.append(f"{pic.prefix}_{pic.want}.jpg")
        return set_text(p0, f"📷 사진 대기\n{pic.prefix}_{pic.want}.jpg")
    img = re.sub(r"</w:p>$", drawing(path) + "</w:p>", re.sub(r"<w:r(?: [^>]*)?>.*?</w:r>", "", p0, flags=re.S))
    return img + (set_text(p0, pic.caption) if pic.caption else "")


def row(cells, header=False, height=None):
    src = ROW_H if header else ROW_D
    tcs = re.findall(r"<w:tc>.*?</w:tc>", src, re.S)
    out = []
    for tc, c in zip(tcs, cells):
        tcpr = re.search(r"<w:tcPr>.*?</w:tcPr>", tc, re.S).group(0).replace('<w:vMerge w:val="restart"/>', "").replace("<w:vMerge/>", "")
        p0 = re.search(r"<w:p(?: [^>]*)?>.*?</w:p>", tc, re.S).group(0)
        if isinstance(c, Pic):
            paras = pic_cell(p0, c)
        else:
            paras = "".join(set_text(p0, line) for line in (c.split("\n\n") if c else [""]))
        out.append(f"<w:tc>{tcpr}{paras}</w:tc>")
    trpr = re.search(r"<w:trPr>.*?</w:trPr>", src, re.S).group(0)
    if height: trpr = re.sub(r'<w:trHeight w:val="\d+"', f'<w:trHeight w:val="{height}"', trpr)
    return f"<w:tr>{trpr}{''.join(out)}</w:tr>"


def table(hdr, data, widths=None, heights=1200):
    tblpr = re.search(r"<w:tblPr>.*?</w:tblPr>", TBL, re.S).group(0)
    w = widths or [700, 4080, 2580, 2000]
    grid = "<w:tblGrid>" + "".join(f'<w:gridCol w:w="{v}"/>' for v in w) + "</w:tblGrid>"
    t = f"<w:tbl>{tblpr}{grid}{row(hdr, header=True)}{''.join(row(d, height=heights) for d in data)}</w:tbl>"
    i = 0
    def repl(m):
        nonlocal i
        v = w[i % len(w)]; i += 1
        return f'<w:tcW w:w="{v}" w:type="dxa"/>'
    return re.sub(r"<w:tcW [^>]*/>", repl, t)


# ───────────── 내용 (보고서_인계.md · 허브 설명서 §8 실측) ─────────────
D = "2026년 10월 31일"
B = [para(P_TITLE, "세팅 작업 보고서"), para(P_SUB, f"일시 : {D}  ·  1번배 OAK-1 PoE 첫 연결·시험")]
B.append(para(P_H1, "1. 작업 개요"))
for s in ["일시 / 장소 : 2026-10-31 / ____",
          "참석 : ____",
          "목표 : OAK-1 PoE 를 1번배 노트북에 연결 → 컬러 토픽·fps 확인 → 센서 전부와 10분 연속 → 화각·각도 실측 → 도킹 도형 시험 준비",
          "전원 : 배용 DC 인젝터(PSE5502G 계열) 미도착 → 벤치용 AC PoE 어댑터(RISUNIC RP028-4800320Z)로 급전. 배 탑재 결선 아님"]:
    B.append(para(P_BULLET, s))
B.append(para(P_H1, "2. 결과 요약"))
for s in ["연결 ✅ : 169.254.1.222 ping 손실 0 %, depthai-ros 2.31.1 연결 13~17 s, /oak/rgb/image_raw 640×360 30 fps",
          "10분 연속 ✅ : OAK 30.0 · D455 30.0 · 라이다 11.2 Hz · IMU 100 Hz · GPS 1 Hz · 비전 29.5 fps, USB 끊김 0, CPU 여유 83 %",
          "화각 ✅ : 63.9° (640×360, ISP 1/3). camera_info 63.87° 와 0.01° 차. 사양 69° 는 센서 전체 값",
          "각도 ✅ : 마스트 위 2.0 m, 좌 1.00 m 원 +26.99° (기대 +27.63°) — 부호·크기 1° 안",
          "도킹 도형 ⏳ : 도구·A4 도형 준비 완료, 본시험 전"]:
    B.append(para(P_BULLET, s))
B.append(para(P_H1, "3. 다음 작업 계획"))
for s in ["도킹 도형 본시험 (보고서_인계 §6 절차, 2 m · 3 m)",
          "DC 인젝터 도착 → 배터리 A 분기 2 A 퓨즈 결선 → 기가비트 링크 · 1280×720 fps 재측정",
          "D455 와 비교 후 카메라 확정 → vision.yaml · ssf_tools.yaml image_topic 전환 (별도 계약)"]:
    B.append(para(P_BULLET, s))

B.append(para(P_TITLE, "SSF 세팅 활동 보고").replace("<w:pPr>", "<w:pPr><w:pageBreakBefore/>", 1))
B.append(para(P_SUB, D))
hdr = ["일시", "활동사진", "활동내용", "특이사항"]

B.append(para(P_H1, "전원 · 연결 — OAK 벤치 급전 · 노트북 네트워크"))
B.append(table(hdr, [
    ["10/31", Pic("00", "벤치용_PoE어댑터_RISUNIC", "벤치용 AC PoE 어댑터 라벨"),
     "RISUNIC RP028-4800320Z (48 V 0.32 A, 15.36 W)로 OAK 급전\n\nPoE 포트 → OAK, LAN 포트 → 노트북 랜 어댑터",
     "배 탑재품 아님(AC 220 V)\n\n10/100 전용 → 링크 100 Mb 반이중"],
    ["", Pic("05", "PoE어댑터_OAK_랜선연결"),
     "노트북 nmcli oak-poe 169.254.1.10/16 (AX88179 enxc8a362944948)\n\nping 169.254.1.222 손실 0 %, TCP 11490 열림",
     "링크 오류·충돌 0"],
    ["", Pic("06", "노트북_랜어댑터_허브"),
     "노트북: 허브(UH308) 아래 라이다·IMU·GPS·D455, 랜 어댑터에 OAK",
     "허브 계통은 10-28 결선 (아래 표)"],
]))
B.append(para(P_EMPTY, ""))

B.append(para(P_H1, "소프트웨어 — OAK 드라이버 · 화각 · 각도"))
B.append(table(hdr, [
    ["10/31", Pic("07", "마스트위_OAK외관"),
     "ros-humble-depthai-ros (depthai 2.31.1), 연결 13~17 s\n\n/oak/rgb/image_raw (bgr8, RELIABLE)\n\n640×360 MJPEG 30 fps / 1280×720 비압축 8 fps",
     "1280×720 은 100 Mb 포화 — 기가 인젝터 후 재측정"],
    ["", Pic("01", "화각_잘린화면_40도", "기본 설정: 가운데만 잘림, 원 간격 436.6 px → 40.3°"),
     "드라이버 기본 설정으로 640×360 요청 → 화면 가운데만 잘려 화각 ~40°",
     "i_isp_num 1 / i_isp_den 3 필요\n\ntools/oak/oak_640x360_full.yaml"],
    ["", Pic("02", "화각_전체화면", "ISP 1/3 전체 화면: 원 간격 213.8 px → 63.9°"),
     "빨간 원 2장 중심 간격 1.00 m, 렌즈–벽 2.40 m\n\n60장 흔들림 0.27 px → 화각 63.9°",
     "camera_info 63.87° 와 0.01° 차\n\n렌즈 왜곡 오차 최대 약 1°"],
    ["", Pic("03", "각도검증_정중앙_마스트위", "마스트 위 정중앙 원, 주황선 = 게이트 높이 띠"),
     "벽 2.0 m, 게이트 노드 /red_angle\n\n정중앙 +1.34° (원이 5 cm 왼쪽)\n\n좌 1.00 m +26.99° (기대 +27.63°)\n\n좌 1.10 m +28.91° (기대 +29.83°)",
     "왼쪽 = 양수 부호 확인\n\n오차 1° 안"],
    ["", Pic("04", "도킹판정_예비_원", "도킹 판정 도구 예비 화면"),
     "도킹 판정 도구 예비: 화면 왼쪽 끝 빨간 원(605 px²)이 30장 중 2장 사각형으로 오판",
     "원형도 0.83 이 기준 0.82 에 걸림 — 본시험에서 확인 ⏳"],
]))
B.append(para(P_EMPTY, ""))

B.append(para(P_H1, "시험 — 센서 전부 + OAK 10분 연속 · 복구 시간"))
B.append(table(["항목", "측정값", "기준 / 판정", "비고"], [
    ["10분 연속", "OAK 30.0 fps · D455 30.0 fps · 라이다 11.2 Hz · IMU 100 Hz · GPS 1 Hz · 비전 처리 29.5 fps", "USB 끊김 0 · 노드 사망 0 ✅", "CPU 여유 83 %\n\n최대 공백 IMU 0.34 s · 비전 0.33 s 한 번"],
    ["정상 재시작", "14~17 s 에 비전 복구", "✅", ""],
    ["강제 종료 후", "~85~90 s 재연결 안 됨", "⚠️ 경기 중 드라이버 사망 = 약 1.5 분 비전 공백", "카메라가 옛 연결을 붙잡음, DEPTHAI_WATCHDOG 로도 안 줄어듦"],
], widths=[1400, 3600, 2400, 1960], heights=500))
B.append(para(P_EMPTY, ""))

B.append(para(P_H1, "참고 — 허브 계통 결선 (10-28, 1번배)"))
B.append(table(hdr, [
    ["10/28", Pic("08", "퓨즈홀더_배터리분기점"),
     "배터리 A 분기 → 3 A 퓨즈 → SZWENGAO 5 V 모듈 → UH308\n\n5 V 출력 정상 · 허브 LED ✅",
     "처음엔 허브 아래 장치 0 — UH308 포트별 전원 버튼이 꺼져 있었음"],
    ["", Pic("09", "5V모듈_UH308결선"),
     "허브 아래 라이다 10.4 Hz · IMU 100 Hz · D455 30 fps · GPS 1 Hz 인식\n\n스러스터 1600 양쪽 30 s 부하 중 4종 동시 — USB 끊김 0",
     "배터리 A 를 빼도 허브는 노트북 USB 전원으로 넘어가 유지 (15 s 확인)"],
]))
B.append(para(P_EMPTY, ""))

B.append(para(P_H1, "탑재장비 제원 (이번 시험 관련)"))
B.append(table(["장비", "모델", "전원 · 인터페이스", "비고"], [
    ["비전 카메라", "Luxonis OAK-1 PoE FF", "802.3af Class 3 · 이더넷 · IP67", "실측 화각 63.9° (640×360, ISP 1/3), 4~7.5 W"],
    ["벤치 PoE 어댑터", "RISUNIC RP028-4800320Z", "AC 100~240 V → 48 V 15.36 W · 10/100", "10-31 시험용, 배 탑재 안 함"],
    ["PoE 인젝터 (배용)", "PSE5502G 계열", "DC 12~48 V → 48 V af/at · 기가", "⏳ 미도착"],
    ["이더넷 어댑터", "NETmate NM-UCL02 (AX88179)", "USB-C → RJ45 1G", "노트북 직결"],
    ["USB 허브", "ipTIME UH308 유전원", "5 V (SZWENGAO 5V 5A, 3 A 퓨즈)", "라이다·IMU·GPS·D455"],
], widths=[1800, 2800, 2600, 2160], heights=500))
B.append(para(P_EMPTY, ""))

B.append(para(P_H1, "결과 체크리스트"))
for s in ["☑ OAK LED · ping 169.254.1.222", "☑ /oak/rgb/image_raw 30 fps", "☑ 센서 전부 + OAK 10분 연속, USB 끊김 0",
          "☑ 화각 실측 63.9°", "☑ 각도 부호·크기 1° 안", "☐ 도킹 도형 본시험", "☐ DC 인젝터 배 결선 (미도착)",
          "☐ vision.yaml image_topic OAK 전환 (D455 비교 후)"]:
    B.append(para(P_BULLET, s))
B.append(para(P_H1, "문제 · 조치"))
B.append(table(["#", "증상", "원인 / 조치", "상태"], [
    ["1", "640×360 요청 시 화각 ~40°", "드라이버가 가운데만 잘라냄 → ISP 1/3 축소 설정(oak_640x360_full.yaml)", "해결"],
    ["2", "드라이버 강제 종료 후 85~90 s 재연결 불가", "카메라가 옛 연결 유지. 정상 종료로 운용, 경기 중 공백 위험 기록", "미해결"],
    ["3", "가장자리 작은 빨간 원 2/30 사각 오판", "원형도 기준 경계값. 본시험에서 거리별 확인", "⏳"],
    ["4", "1280×720 8 fps", "벤치 어댑터 100 Mb 한계. 기가 인젝터 후 재측정", "⏳"],
], widths=[600, 3000, 4000, 1760], heights=500))
B.append(SECT)

new = x[:x.index("<w:body>") + 8] + "".join(B) + x[x.index("</w:body>"):]
rels = z.read("word/_rels/document.xml.rels").decode()
rels = rels.replace("</Relationships>", "".join(
    f'<Relationship Id="{rid}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="{n}"/>'
    for n, _, rid in MEDIA) + "</Relationships>")
with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as zo:
    for n in z.namelist():
        if n == "word/document.xml": data = new.encode("utf8")
        elif n == "word/_rels/document.xml.rels": data = rels.encode("utf8")
        else: data = z.read(n)
        zo.writestr(n, data)
    for n, b, _ in MEDIA:
        zo.writestr("word/" + n, b)
print("wrote", os.path.normpath(OUT), f"— 사진 {len(MEDIA)}장")
for m in MISSING: print("  사진 대기:", m)
