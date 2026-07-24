"""
비전 노드 모드 게이팅 — ROS/OpenCV 비의존 순수 로직. 시간은 주입받는다.

왜 있나 (CLAUDE.md 3-4):
  작년엔 `subscriber_mode_manager` 가 /wp_mode 마다 비전 노드를 **subprocess 로 죽였다 살렸다.**
    self._child = subprocess.Popen(["ros2","run",pkg,exe], preexec_fn=os.setsid)
    os.killpg(os.getpgid(self._child.pid), signal.SIGINT); time.sleep(0.5) ...
  문제:
    · 모드 전환마다 비전이 **몇 초간 완전히 멈춘다** (kill sleep 0.5~1.5s + `ros2 run` 기동)
    · subprocess 가 좀비로 남으면 **카메라가 잠긴다**
    · 추적기(tracker)를 쓸 수 없다 — 노드가 죽으면 트랙이 전부 날아간다
    · 파라미터가 안 닿는다 → 매니저가 --ros-args 로 중계하는 이중 배선이 필요했다
  → 노드는 **항상 살아있고**, 자기 담당 모드가 아닐 때 **처리와 발행을 멈춘다.**
    각 노드가 자기 모드를 소유한다(미션 노드 ship_gate/dock/turn/back 이 이미 쓰는 패턴).

🚨 발행 게이팅은 선택이 아니라 필수다.
  `dock` 과 `turn` 은 **둘 다 `/image_angle` 을 발행**한다. 상주시키면서 게이팅을 안 하면
  **한 토픽에 발행자 2개**가 되어 두 값이 번갈아 나온다. 에러는 안 난다 —
  이 프로젝트가 반복해 당한 침묵 실패다(도킹 mode 9 vs 7, ship_back 이 'ship_turn' 으로 등록).

🚨 모드를 모를 때(부팅 직후 /wp_mode 미수신, 또는 stale)는 **비활성**이다.
  "모르면 발행하지 않는다" — 틀린 방위를 내보내느니 침묵이 낫다.
  단 그 상태가 조용하면 안 되므로 호출부가 이유를 로그로 낸다.
"""


class ModeGate:
    """
    /wp_mode 로 활성/비활성을 판정한다.

    사용:
        gate = ModeGate(active_modes=[0, 1], stale_sec=2.0)
        gate.update(wp_mode, t)          # /wp_mode 콜백에서
        if not gate.is_active(t): return # 이미지 콜백 맨 앞에서
    """

    # 비활성 사유 (진단용 — 조용히 멈추지 않으려고 남긴다)
    R_NO_MODE = "no_wp_mode"       # 부팅 후 /wp_mode 를 한 번도 못 받음
    R_STALE = "wp_mode_stale"      # 받았었는데 끊김 (FSM 사망 가능성)
    R_OTHER_MODE = "other_mode"    # 정상 — 지금은 내 차례가 아니다

    def __init__(self, active_modes, *, stale_sec=2.0):
        self.active_modes = {int(m) for m in active_modes}
        self.stale_sec = float(stale_sec)
        self._mode = None
        self._t = None

    def update(self, wp_mode, t):
        """/wp_mode 수신. 정수가 아니면 무시한다(슬롯 오염 방지)."""
        try:
            m = int(wp_mode)
        except (TypeError, ValueError):
            return False
        self._mode = m
        self._t = t
        return True

    def state(self, t):
        """(활성?, 사유) 를 돌려준다. 사유는 비활성일 때만 의미가 있다."""
        if self._t is None:
            return False, self.R_NO_MODE
        if (t - self._t) > self.stale_sec:
            # FSM 이 죽었을 수 있다. 옛 모드로 계속 발행하면 배가 지난 미션을 계속 한다.
            return False, self.R_STALE
        if self._mode not in self.active_modes:
            return False, self.R_OTHER_MODE
        return True, None

    def is_active(self, t):
        return self.state(t)[0]

    @property
    def mode(self):
        """마지막으로 받은 wp_mode. 아직 못 받았으면 None."""
        return self._mode


def check_publisher_conflicts(node_specs):
    """
    상주 노드들이 같은 토픽을 겹치는 모드에서 발행하지 않는지 정적 검사.

    node_specs: {노드이름: (활성모드집합, 발행토픽집합)}
    반환: 충돌 목록 [(토픽, 모드, [노드…])]. 빈 리스트면 안전.

    🚨 이 검사가 있는 이유: dock 과 turn 이 둘 다 /image_angle 을 발행한다.
       모드가 겹치는 순간 한 토픽에 발행자 2개가 되고, **에러 없이** 값이 섞인다.
    """
    conflicts = []
    all_modes = set()
    for modes, _topics in node_specs.values():
        all_modes |= set(modes)

    for mode in sorted(all_modes):
        by_topic = {}
        for name, (modes, topics) in node_specs.items():
            if mode not in modes:
                continue
            for tp in topics:
                by_topic.setdefault(tp, []).append(name)
        for tp, names in sorted(by_topic.items()):
            if len(names) > 1:
                conflicts.append((tp, mode, sorted(names)))
    return conflicts
