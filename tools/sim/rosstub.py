"""rclpy 스텁 — 실제 노드 파일을 ROS 없이 import 해 콜백·타이머를 손으로 돌리기 위한 것.

  from rosstub import install, Clock, load_params, load_node
  install()                                  # sys.modules 에 rclpy/std_msgs/sensor_msgs 스텁 등록
  P = load_params('src/x/config/x.yaml', '/x')
  mod = load_node('src/x/x/x.py', 'x_real', extra_path='src/x')   # time.monotonic 을 Clock 으로 바꿔 준다
  node = mod.SomeNode(params=P)  ← Node 스텁은 생성 직후 declare_parameter 에 PARAMS[name] 을 준다

노드 코드는 한 줄도 바꾸지 않는다(dock_sim.py 와 같은 방식). 시뮬 전용 — 실행 환경 아님.
"""
import importlib.util
import os
import sys
import types

PARAMS = {}          # 노드 이름 → {param: value}


class Clock:
    t = 0.0

    @staticmethod
    def monotonic():
        return Clock.t


class Param:
    def __init__(self, v):
        self.value = v


class Logger:
    def __init__(self):
        self.lines = []

    def _log(self, lvl, msg, **kw):
        self.lines.append((lvl, msg))

    def info(self, m, **kw): self._log("INFO", m)
    def warn(self, m, **kw): self._log("WARN", m)
    def warning(self, m, **kw): self._log("WARN", m)
    def error(self, m, **kw): self._log("ERROR", m)
    def debug(self, m, **kw): self._log("DEBUG", m)


class Pub:
    def __init__(self, topic):
        self.topic = topic
        self.last = None
        self.count = 0

    def publish(self, msg):
        self.last = msg
        self.count += 1


class Node:
    def __init__(self, name):
        self._name = name
        self._params = dict(PARAMS.get(name, {}))
        self._logger = Logger()
        self.pubs = {}

    def declare_parameter(self, name, default=None):
        return Param(self._params.get(name, default))

    def create_publisher(self, typ, topic, qos):
        p = Pub(topic)
        self.pubs[topic] = p
        return p

    def create_subscription(self, *a, **k):
        return None

    def create_timer(self, *a, **k):
        return None

    def get_logger(self):
        return self._logger

    def get_name(self):
        return self._name

    def destroy_node(self):
        pass


class Msg:
    def __init__(self, data=None):
        self.data = [] if data is None else data


def install():
    if "rclpy" in sys.modules and getattr(sys.modules["rclpy"], "_ssf_stub", False):
        return
    rclpy = types.ModuleType("rclpy"); rclpy._ssf_stub = True
    rclpy.init = lambda *a, **k: None; rclpy.ok = lambda: True; rclpy.shutdown = lambda: None; rclpy.spin = lambda *a, **k: None
    node = types.ModuleType("rclpy.node"); node.Node = Node
    ex = types.ModuleType("rclpy.executors"); ex.MultiThreadedExecutor = object
    qos = types.ModuleType("rclpy.qos")
    for n in ("QoSProfile", "ReliabilityPolicy", "HistoryPolicy", "DurabilityPolicy"):
        setattr(qos, n, type(n, (), {"__init__": lambda self, **k: None, "BEST_EFFORT": 1, "RELIABLE": 2,
                                      "KEEP_LAST": 1, "VOLATILE": 1, "TRANSIENT_LOCAL": 2}))
    std = types.ModuleType("std_msgs"); stdm = types.ModuleType("std_msgs.msg")
    for n in ("Float32", "Float64", "Int32", "Bool", "Float32MultiArray", "String", "Int32MultiArray", "Float64MultiArray"):
        setattr(stdm, n, type(n, (Msg,), {}))
    sen = types.ModuleType("sensor_msgs"); senm = types.ModuleType("sensor_msgs.msg"); senm.LaserScan = type("LaserScan", (), {})
    for k, v in {"rclpy": rclpy, "rclpy.node": node, "rclpy.executors": ex, "rclpy.qos": qos,
                 "std_msgs": std, "std_msgs.msg": stdm, "sensor_msgs": sen, "sensor_msgs.msg": senm}.items():
        sys.modules[k] = v


def load_params(yaml_path, node_key):
    import yaml
    with open(yaml_path, encoding="utf-8") as f:
        d = yaml.safe_load(f)
    p = d[node_key]["ros__parameters"]
    PARAMS[node_key.strip("/")] = p
    return p


def load_node(py_path, mod_name, extra_path=None):
    """노드 파일을 모듈로 로드하고 그 모듈의 time.monotonic 을 Clock 으로 바꾼다."""
    if extra_path and extra_path not in sys.path:
        sys.path.insert(0, extra_path)
    spec = importlib.util.spec_from_file_location(mod_name, py_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if hasattr(mod, "time"):
        mod.time = types.SimpleNamespace(monotonic=Clock.monotonic, time=Clock.monotonic, sleep=lambda s: None)
    return mod
