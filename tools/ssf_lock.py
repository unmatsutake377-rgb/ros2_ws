"""배 노트북 안에서 ssf 작업을 한 번에 하나씩 — 앱·터미널 ssf·다른 MacBook 이 동시에 해도 겹치지 않게.

  with locked(home, "param"):  ...   # yaml 고치기(읽기 → 바꾸기 → 쓰기 → 검사 → 기록)
  with locked(home, "wp_B"):   ...   # 녹화 도구에 키 보내기
  with locked(home, "cam_B"):  ...   # 영상 송신 켜기·끄기

왜(2026-11-10 동시 시험): 잠금이 없을 때 같은 yaml 의 다른 값 두 개를 동시에 적용하면 15번 중 8번 한쪽이
소리 없이 사라졌고(둘 다 ✅), 두 곳에서 동시에 '찍기'를 누르면 두 번째 Enter 가 mode 질문을 기본값으로 넘겼다.
- 다른 프로세스끼리: fcntl.flock(파일 잠금). 프로세스가 죽으면 OS 가 풀어 준다 — 남는 잠금 없음.
- 같은 프로세스의 다른 스레드끼리: threading.Lock.
- 같은 스레드가 다시 잡으면 통과(재진입 — apply 안에서 부르는 undo 등).
"""
import contextlib
import fcntl
import os
import threading
import time

_local = threading.local()          # 스레드별 {경로: 깊이}
_tlocks = {}
_tguard = threading.Lock()


class LockTimeout(RuntimeError):
    pass


def _tlock(path):
    with _tguard:
        return _tlocks.setdefault(path, threading.Lock())


@contextlib.contextmanager
def locked(home, name, timeout=20.0):
    d = os.path.join(home, "ssf_logs")
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, f".ssf_{name}.lock")
    depth = getattr(_local, "depth", None)
    if depth is None:
        depth = _local.depth = {}
    if depth.get(path):
        depth[path] += 1
        try:
            yield
        finally:
            depth[path] -= 1
        return
    msg = f"다른 ssf 작업이 {int(timeout)} s 넘게 끝나지 않는다({name}) — 잠시 뒤 다시"
    end = time.time() + timeout
    tl = _tlock(path)
    if not tl.acquire(timeout=timeout):
        raise LockTimeout(msg)
    f = None
    try:
        f = open(path, "a+")
        while True:
            try:
                fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.time() > end:
                    raise LockTimeout(msg)
                time.sleep(0.05)
        depth[path] = 1
        try:
            yield
        finally:
            depth[path] = 0
            fcntl.flock(f, fcntl.LOCK_UN)
    finally:
        if f:
            f.close()
        tl.release()
