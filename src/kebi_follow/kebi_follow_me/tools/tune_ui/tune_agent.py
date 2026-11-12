#!/usr/bin/env python3
"""follow-me 조정 중계 (로봇에서 실행). 표준입출력으로 JSON 한 줄씩 주고받는다.

tune_server.py 가 이 프로그램을 (보통 SSH 로) 띄우고 stdin/stdout 을 잇는다. 로봇에 포트를 열 필요가 없다.
  입력  {"op": "get" | "set" | "cmd", "id": N, "values": {...}, "cmd": "start" | "stop" | "estop"}
  출력  {"type": "hello" | "params" | "result" | "tel" | "log", ...}
조정 대상: follow_controller·leg_tracker 파라미터 중 read_only 가 아니고 범위(floating_point_range)가 있는 것.
  이름: follow_controller 는 그대로(예 max_linear_speed), leg_tracker 는 "leg_tracker/" 를 붙인다(예 leg_tracker/ambiguity.radius).
"""
import json
import math
import queue
import sys
import threading
import time

import rclpy
from follow_msgs.msg import UiClient
from geometry_msgs.msg import Twist
from kebi_follow_msgs.msg import FollowState, FollowTarget
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import DescribeParameters, GetParameters, ListParameters, SetParametersAtomically
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

NODE = "/follow_controller"       # 없으면 조정 불가 (주 노드)
NODES = {"": NODE, "leg_tracker/": "/leg_tracker"}   # 화면 이름 접두어 → 노드
TEL_HZ = 10.0
SPEED_WIN_S = 0.2        # 실측 속도 창 [s] (popo 의 /odom twist 는 늘 0 → 자세 변화로 계산)
CMD_CONFIRM_S = 2.0      # START/STOP 뒤 FSM 확인 대기 [s]
ESTOP_ZEROS = 20         # 비상정지: /cmd_vel 0 을 20 Hz 로 1 s
SERVICE_CHECK_S = 1.0    # follow_controller 가 떴는지 확인하는 주기 [s]
FSM_IDLE = 0
CMD = {"start": 1, "stop": 3, "estop": 3}


def out(obj):
    sys.stdout.write(json.dumps(obj, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def as_number(v):
    if v.type == ParameterType.PARAMETER_DOUBLE:
        return v.double_value
    if v.type == ParameterType.PARAMETER_INTEGER:
        return float(v.integer_value)
    return None


class Agent:
    def __init__(self):
        self.n = rclpy.create_node("kebi_follow_tune_agent")
        mk = self.n.create_client
        self.cl = {pre: {"list": mk(ListParameters, node + "/list_parameters"),
                         "desc": mk(DescribeParameters, node + "/describe_parameters"),
                         "get": mk(GetParameters, node + "/get_parameters"),
                         "set": mk(SetParametersAtomically, node + "/set_parameters_atomically")}
                   for pre, node in NODES.items()}
        self.c_get = self.cl[""]["get"]
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.ui = self.n.create_publisher(UiClient, "/follow/ui/client", rel)
        self.vel = self.n.create_publisher(Twist, "/cmd_vel", 1)
        self.st, self.tg = None, None
        self.odom = []           # (stamp, x, y, yaw)
        self.n.create_subscription(FollowState, "/kebi_follow/state", self._on_state, rel)
        self.n.create_subscription(FollowTarget, "/kebi_follow/target", lambda m: setattr(self, "tg", m), rel)
        self.n.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.ranges = {}         # 이름 → [lo, hi, 설명]
        self.inbox = queue.Queue()
        self.pending = []        # (deadline, future, finisher)
        self.waiters = []        # (deadline, pred, finisher) : FSM 확인 등
        self.n.create_timer(1.0 / TEL_HZ, self._tel)

    # ------------------------------------------------------------- 입력
    def _on_state(self, m):
        self.st = m

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
        while len(self.odom) > 2 and self.odom[-1][0] - self.odom[0][0] > SPEED_WIN_S:
            self.odom.pop(0)

    def _speed(self):
        if len(self.odom) < 2 or self.odom[-1][0] - self.odom[0][0] < 0.05:
            return None, None
        a, b = self.odom[0], self.odom[-1]
        dt = b[0] - a[0]
        v = ((b[1] - a[1]) * math.cos(b[3]) + (b[2] - a[2]) * math.sin(b[3])) / dt
        return v, math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])) / dt

    def _tel(self):
        st, tg = self.st, self.tg
        av, aw = self._speed()
        out({"type": "tel", "t": time.time(),
             "fsm": st.fsm if st else None, "stop": st.stop_reasons if st else 0,
             "cmd_v": st.cmd_v if st else None, "cmd_w": st.cmd_w if st else None,
             "v_limit": st.v_limit if st else None, "shadow": bool(st.shadow) if st else None,
             "obs_scan": min(st.scan_obstacle_distance, 9.9) if st else None,
             "obs_depth": min(st.depth_obstacle_distance, 9.9) if st else None,
             "tgt": tg.status if tg else None, "tgt_id": tg.track_id if tg else None,
             "range": tg.range if tg and tg.status in (1, 2, 3) else None,
             "bearing_deg": math.degrees(tg.bearing) if tg and tg.status in (1, 2, 3) else None,
             "cam": bool(tg.camera_confirmed) if tg else None, "amb": bool(tg.ambiguous) if tg else None,
             "act_v": av, "act_w": aw})

    # ------------------------------------------------------------- 파라미터
    def _call(self, client, req, finisher, timeout=3.0):
        if not client.service_is_ready():
            finisher(None)
            return
        self.pending.append((time.monotonic() + timeout, client.call_async(req), finisher))

    def get(self, rid):
        """모든 노드의 조정 가능 값을 읽어 한 번에 알린다. 주 노드(follow_controller)가 없으면 실패."""
        acc = {"values": {}, "ranges": {}, "left": len(NODES), "main_ok": False}

        def node_done(pre, ok, ranges=None, values=None):
            if ok:
                acc["ranges"].update(ranges)
                acc["values"].update(values)
                if pre == "":
                    acc["main_ok"] = True
            acc["left"] -= 1
            if acc["left"] > 0:
                return
            if not acc["main_ok"]:
                out({"type": "result", "id": rid, "ok": False, "reason": "follow_controller 없음 (컨테이너가 떠 있나?)"})
                return
            self.ranges = acc["ranges"]
            out({"type": "params", "values": acc["values"], "ranges": acc["ranges"]})
            out({"type": "result", "id": rid, "ok": True})

        for pre, c in self.cl.items():
            self._get_node(pre, c, node_done)

    def _get_node(self, pre, c, done):
        def got_list(res):
            if res is None:
                done(pre, False)
                return
            names = list(res.result.names)
            self._call(c["desc"], DescribeParameters.Request(names=names), lambda r: got_desc(r, names))

        def got_desc(res, names):
            if res is None:
                done(pre, False)
                return
            ranges = {d.name: [d.floating_point_range[0].from_value, d.floating_point_range[0].to_value, d.description]
                      for d in res.descriptors if not d.read_only and d.floating_point_range}
            tun = sorted(ranges)
            self._call(c["get"], GetParameters.Request(names=tun), lambda r: got_vals(r, tun, ranges))

        def got_vals(res, tun, ranges):
            if res is None:
                done(pre, False)
                return
            done(pre, True, {pre + n: r for n, r in ranges.items()},
                 {pre + n: as_number(v) for n, v in zip(tun, res.values)})

        self._call(c["list"], ListParameters.Request(), got_list)

    def set(self, rid, values):
        """노드별로 나눠 원자적으로 바꾼다. 하나라도 거부되면 실패로 알린다 (다른 노드 것은 이미 적용됐을 수 있다)."""
        groups = {}
        for k, v in values.items():
            pre = next((p for p in sorted(NODES, key=len, reverse=True) if p and k.startswith(p)), "")
            pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))
            groups.setdefault(pre, []).append(Parameter(name=k[len(pre):], value=pv))
        acc = {"left": len(groups), "ok": True, "reasons": []}

        def done(pre, res):
            if res is None:
                acc["ok"] = False
                acc["reasons"].append(f"{NODES[pre]} 응답 없음")
            elif not res.result.successful:
                acc["ok"] = False
                acc["reasons"].append(f"{NODES[pre]}: {res.result.reason}")
            acc["left"] -= 1
            if acc["left"] == 0:
                out({"type": "result", "id": rid, "ok": acc["ok"], "reason": "; ".join(acc["reasons"])})
                self.get(-1)   # 바뀐 값 다시 알림

        if not groups:
            out({"type": "result", "id": rid, "ok": True, "reason": ""})
            return
        for pre, ps in groups.items():
            self._call(self.cl[pre]["set"], SetParametersAtomically.Request(parameters=ps),
                       lambda r, pre=pre: done(pre, r))

    # ------------------------------------------------------------- START / STOP
    def cmd(self, rid, what):
        if what not in CMD:
            out({"type": "result", "id": rid, "ok": False, "reason": f"모르는 명령 {what}"})
            return
        msg = UiClient()
        msg.cmd = CMD[what]
        self.ui.publish(msg)
        if what == "estop":
            for _ in range(ESTOP_ZEROS):
                self.vel.publish(Twist())
                time.sleep(0.05)

        def ok_now():
            if self.st is None:
                return False
            return self.st.fsm != FSM_IDLE if what == "start" else self.st.fsm == FSM_IDLE

        tries = [1]

        def fin(ok):
            if not ok and tries[0] < 3:      # 매칭 직후 유실 대비 재전송 (컨트롤러는 중복 START 를 무시)
                tries[0] += 1
                self.ui.publish(msg)
                self.waiters.append((time.monotonic() + CMD_CONFIRM_S, ok_now, fin))
                return
            out({"type": "result", "id": rid, "ok": bool(ok),
                 "reason": "" if ok else "FSM 이 바뀌지 않음 (컨테이너·연결 확인)"})

        self.waiters.append((time.monotonic() + CMD_CONFIRM_S, ok_now, fin))

    # ------------------------------------------------------------- 반복
    def step(self):
        while True:
            try:
                req = self.inbox.get_nowait()
            except queue.Empty:
                break
            if req is None:
                return False
            op, rid = req.get("op"), req.get("id", 0)
            if op == "get":
                self.get(rid)
            elif op == "set":
                self.set(rid, req.get("values", {}))
            elif op == "cmd":
                self.cmd(rid, req.get("cmd", ""))
            else:
                out({"type": "result", "id": rid, "ok": False, "reason": f"모르는 op {op}"})
        now = time.monotonic()
        keep = []
        for dl, fut, fin in self.pending:
            if fut.done():
                fin(fut.result())
            elif now > dl:
                fin(None)
            else:
                keep.append((dl, fut, fin))
        self.pending = keep
        keep = []
        for dl, pred, fin in self.waiters:
            if pred():
                fin(True)
            elif now > dl:
                fin(False)
            else:
                keep.append((dl, pred, fin))
        self.waiters = keep
        return True


def read_stdin(q):
    for line in sys.stdin:
        line = line.strip()
        if line:
            try:
                q.put(json.loads(line))
            except json.JSONDecodeError:
                out({"type": "log", "msg": f"bad json: {line[:80]}"})
    q.put(None)   # 서버가 끊기면 끝낸다


def main():
    rclpy.init()
    a = Agent()
    threading.Thread(target=read_stdin, args=(a.inbox,), daemon=True).start()
    out({"type": "hello", "node": NODE})
    last_check, was_ready = 0.0, {}
    while rclpy.ok():
        rclpy.spin_once(a.n, timeout_sec=0.02)
        now = time.monotonic()
        if now - last_check > SERVICE_CHECK_S:
            # 컨테이너(노드)가 (다시) 뜨면 값을 자동으로 읽고, 주 노드가 내려가면 목록을 비운다
            last_check = now
            ready = {pre: c["get"].service_is_ready() for pre, c in a.cl.items()}
            if ready[""] and (ready != was_ready or not a.ranges):
                a.get(-1)
            elif was_ready.get("") and not ready[""]:
                a.ranges = {}
                out({"type": "params", "values": {}, "ranges": {}})
            was_ready = ready
        if not a.step():
            break
    a.n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
