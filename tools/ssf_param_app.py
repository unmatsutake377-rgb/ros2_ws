#!/usr/bin/env python3
"""SSF 파라미터 앱 서버 — 배 노트북에서 돌고, MacBook 브라우저가 SSH 터널로 붙는다.

  MacBook:   tools/ssf app a          (= SSH 터널 + 이 서버 시작 + 브라우저 열기. B번배는 b, 둘 다 both)
  배 노트북: python3 tools/ssf_param_app.py --boat A --port 8765   (직접 띄울 때)

화면은 tools/ssf_param_app.html 하나. 값 읽기·고치기는 전부 ssf_param_core.Repo 가 한다(터미널 ssf 와 같은 엔진).

보안
  · 127.0.0.1 에만 묶는다 — 밖에서는 SSH 터널(-L)로만 들어온다. Tailscale 주소로 직접 못 연다.
  · Host 헤더가 localhost/127.0.0.1 이 아니면 거부(DNS 리바인딩 방지).
  · 쓰기(POST)는 'X-SSF: 1' 헤더가 있어야 한다(다른 사이트가 브라우저로 몰래 보내는 요청 차단).
  · NTRIP 파일은 엔진이 아예 목록에 안 올린다. 웨이포인트 파일은 '웨이포인트' 탭에서 보기·검사만(쓰기 API 없음).
  · '오늘 시험' 탭(tools/ssf_test_plan.yaml)도 보기만 — 체크 표시는 브라우저 localStorage 에만 남는다.
  · 'launch' 탭은 자율 launch 를 켜고(이미 떠 있거나 boat_boot 안 됐으면 거부) Ctrl+C 로만 끈다(tools/ssf_launch.py).
    모드 전환(자율 출발)은 하지 않는다 — 조종기로만.
ROS 없이 돈다(파이썬 3.8+ · PyYAML).
"""
import argparse
import json
import os
import sys
import threading
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ssf_param_core as core  # noqa: E402

PAGE = os.path.join(HERE, "ssf_param_app.html")
LOCK = threading.Lock()


def make_handler(repo, port):
    allowed_hosts = {f"localhost:{port}", f"127.0.0.1:{port}", "localhost", "127.0.0.1"}

    class H(BaseHTTPRequestHandler):
        server_version = "ssf-param/1"

        def log_message(self, fmt, *args):      # 조용히(터미널에 요청마다 찍지 않는다)
            pass

        def _send(self, code, body, ctype="application/json; charset=utf-8"):
            data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(data)

        def _host_ok(self):
            return self.headers.get("Host", "") in allowed_hosts

        def do_GET(self):
            if not self._host_ok():
                return self._send(403, {"ok": False, "error": "Host 거부"})
            path = self.path.split("?")[0]
            try:
                if path in ("/", "/index.html"):
                    with open(PAGE, "rb") as f:
                        return self._send(200, f.read(), "text/html; charset=utf-8")
                if path == "/api/ping":
                    return self._send(200, {"ok": True, "app": "ssf-param", "boat": repo.requested_boat})
                if path == "/api/state":
                    return self._send(200, dict(repo.state(), ok=True))
                if path == "/api/params":
                    with LOCK:
                        d = repo.params()
                    d["groups"] = [g["name"] for g in repo.catalog.get("groups", [])]
                    return self._send(200, dict(d, ok=True))
                if path == "/api/diff":
                    with LOCK:
                        return self._send(200, {"ok": True, "rows": repo.diff()})
                if path == "/api/log":
                    return self._send(200, {"ok": True, "rows": repo.read_log(100)})
                if path == "/api/waypoints":       # 보기·검사만(웨이포인트 파일은 고치지 않는다)
                    with LOCK:
                        return self._send(200, {"ok": True, "files": repo.waypoints(),
                                                "boat": repo.view_boat()})
                if path == "/api/gps":             # GPS·RTK 상태(보기 전용)
                    return self._send(200, dict(repo.gps(), ok=True))
                if path == "/api/cam":             # D455 영상 송신 상태
                    return self._send(200, dict(repo.cam("status"), ok=True))
                if path == "/api/plan":            # '오늘 시험' 탭(tools/ssf_test_plan.yaml, 보기 전용)
                    return self._send(200, dict(repo.test_plan(), ok=True))
                if path == "/api/launch":          # 자율 launch 상태·로그·boat_boot 확인
                    return self._send(200, dict(repo.launch("status"), ok=True))
                if path == "/api/wp/rec":          # 녹화 화면(tmux) 읽기
                    return self._send(200, dict(repo.wp_rec("status"), ok=True))
            except core.ParamError as e:
                return self._send(200, {"ok": False, "error": str(e)})
            return self._send(404, {"ok": False, "error": "없는 주소"})

        def do_POST(self):
            # 본문을 먼저 읽는다 — 안 읽고 거부하면 클라이언트가 응답 대신 '연결 끊김'을 받는다(macOS)
            try:
                n = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                n = -1
            if n < 0 or n > 200_000:
                self.close_connection = True
                return self._send(413, {"ok": False, "error": "너무 큼"})
            raw = self.rfile.read(n) if n else b""
            if not self._host_ok() or self.headers.get("X-SSF") != "1":
                return self._send(403, {"ok": False, "error": "거부(헤더)"})
            try:
                body = json.loads(raw or b"{}")
            except (ValueError, json.JSONDecodeError):
                return self._send(400, {"ok": False, "error": "JSON 아님"})
            path = self.path.split("?")[0]
            try:
                with LOCK:
                    if path == "/api/set":
                        r = repo.apply(body["id"], body["value"], body.get("course") or None,
                                       source="app", dry_run=bool(body.get("dry_run")), rel=body.get("rel"))
                        return self._send(200, r)
                    if path == "/api/undo":
                        return self._send(200, repo.undo(body["id"], body.get("course") or None, source="app",
                                                         rel=body.get("rel")))
                    if path == "/api/cam/start":
                        return self._send(200, dict(repo.cam("start", body.get("ip")), ok=True))
                    if path == "/api/cam/stop":
                        return self._send(200, dict(repo.cam("stop"), ok=True))
                    if path == "/api/launch/start":
                        return self._send(200, dict(repo.launch("start", course=body.get("course"),
                                                                wp=body.get("wp")), ok=True))
                    if path == "/api/launch/stop":
                        return self._send(200, dict(repo.launch("stop"), ok=True))
                    if path == "/api/wp/rec/start":
                        return self._send(200, dict(repo.wp_rec("start", course=body.get("course"),
                                                                lane=body.get("lane")), ok=True))
                    if path == "/api/wp/rec/send":
                        return self._send(200, dict(repo.wp_rec("send", kind=body.get("kind"),
                                                                arg=body.get("arg")), ok=True))
                    if path == "/api/wp/rec/kill":
                        return self._send(200, dict(repo.wp_rec("kill"), ok=True))
                    if path == "/api/build":
                        return self._send(200, repo.build(body.get("pkgs") or []))
                    if path == "/api/quit":
                        threading.Thread(target=self.server.shutdown, daemon=True).start()
                        return self._send(200, {"ok": True})
            except core.ParamError as e:
                return self._send(200, {"ok": False, "error": str(e)})
            except KeyError as e:
                return self._send(400, {"ok": False, "error": f"빠진 항목 {e}"})
            return self._send(404, {"ok": False, "error": "없는 주소"})

    return H


def stop_old(port):
    """같은 포트에 남은 옛 앱 서버(SSH 가 끊겨 남은 것)를 끈다. 다른 프로그램이면 건드리지 않는다."""
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/ping", timeout=1) as r:
            if json.loads(r.read()).get("app") != "ssf-param":
                return False
        req = urllib.request.Request(f"http://127.0.0.1:{port}/api/quit", data=b"{}", method="POST",
                                     headers={"X-SSF": "1", "Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=2).read()
        return True
    except Exception:      # noqa: BLE001 — 없으면 그냥 진행
        return False


def main():
    ap = argparse.ArgumentParser(description="SSF 파라미터 앱 서버 (배 노트북)")
    ap.add_argument("--boat", required=True, choices=["A", "B", "a", "b"])
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--repo", help="저장소(기본 이 파일의 상위)")
    ap.add_argument("--exit-with-parent", action="store_true",
                    help="부모(SSH 세션)가 끝나면 같이 끈다 — tools/ssf 가 붙인다. SSH 가 끊겨 서버만 남는 것 방지")
    a = ap.parse_args()
    repo = core.Repo(a.repo, a.boat)
    if stop_old(a.port):
        print(f"옛 앱 서버(포트 {a.port})를 껐다")
        import time
        time.sleep(0.5)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(repo, a.port))
    st = repo.state()
    print(f"SSF 파라미터 앱 — {st['boat_ko']} · {st['host']} · {repo.repo}")
    if st["boat_error"]:
        print("⚠️ " + st["boat_error"] + " (보기만 됨, 고치기 거부)")
    print(f"브라우저: http://localhost:{a.port}   (끄기: Ctrl+C)")
    sys.stdout.flush()
    if a.exit_with_parent:
        ppid = os.getppid()

        def watch():
            import time
            while True:
                time.sleep(2)
                if os.getppid() != ppid:       # SSH 세션이 끝나 고아가 됐다
                    srv.shutdown()
                    return
        threading.Thread(target=watch, daemon=True).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
