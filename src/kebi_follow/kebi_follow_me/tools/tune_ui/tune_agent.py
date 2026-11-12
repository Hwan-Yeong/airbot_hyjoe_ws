#!/usr/bin/env python3
"""follow-me 조정 중계 (로봇에서 실행). 표준입출력으로 JSON 한 줄씩 주고받는다.

tune_server.py 가 이 프로그램을 (보통 SSH 로) 띄우고 stdin/stdout 을 잇는다. 로봇에 포트를 열 필요가 없다.
  입력  {"op": "get" | "set" | "cmd", "id": N, "values": {...}, "cmd": "start" | "stop" | "estop"}
  출력  {"type": "hello" | "params" | "result" | "tel" | "log", ...}
조정 대상: follow_controller 파라미터 중 read_only 가 아니고 범위(floating_point_range)가 있는 것.
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

NODE = "/follow_controller"
TEL_HZ = 10.0
SPEED_WIN_S = 0.2        # 실측 속도 창 [s] (popo 의 /odom twist 는 늘 0 → 자세 변화로 계산)
CMD_CONFIRM_S = 2.0      # START/STOP 뒤 FSM 확인 대기 [s]
ESTOP_ZEROS = 20         # 비상정지: /cmd_vel 0 을 20 Hz 로 1 s
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
        self.c_list = mk(ListParameters, NODE + "/list_parameters")
        self.c_desc = mk(DescribeParameters, NODE + "/describe_parameters")
        self.c_get = mk(GetParameters, NODE + "/get_parameters")
        self.c_set = mk(SetParametersAtomically, NODE + "/set_parameters_atomically")
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
        def got_list(res):
            if res is None:
                out({"type": "result", "id": rid, "ok": False, "reason": "follow_controller 없음 (컨테이너가 떠 있나?)"})
                return
            names = list(res.result.names)
            self._call(self.c_desc, DescribeParameters.Request(names=names), lambda r: got_desc(r, names))

        def got_desc(res, names):
            if res is None:
                out({"type": "result", "id": rid, "ok": False, "reason": "describe 실패"})
                return
            self.ranges = {d.name: [d.floating_point_range[0].from_value, d.floating_point_range[0].to_value, d.description]
                           for d in res.descriptors if not d.read_only and d.floating_point_range}
            tun = sorted(self.ranges)
            self._call(self.c_get, GetParameters.Request(names=tun), lambda r: got_vals(r, tun))

        def got_vals(res, tun):
            if res is None:
                out({"type": "result", "id": rid, "ok": False, "reason": "get 실패"})
                return
            vals = {n: as_number(v) for n, v in zip(tun, res.values)}
            out({"type": "params", "values": vals, "ranges": self.ranges})
            out({"type": "result", "id": rid, "ok": True})

        self._call(self.c_list, ListParameters.Request(), got_list)

    def set(self, rid, values):
        ps = []
        for k, v in values.items():
            pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))
            ps.append(Parameter(name=k, value=pv))

        def done(res):
            if res is None:
                out({"type": "result", "id": rid, "ok": False, "reason": "set 응답 없음"})
                return
            r = res.result
            out({"type": "result", "id": rid, "ok": bool(r.successful), "reason": r.reason})
            if r.successful:
                self.get(-1)   # 바뀐 값 다시 알림

        self._call(self.c_set, SetParametersAtomically.Request(parameters=ps), done)

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
    t_ready = time.monotonic()
    announced = False
    while rclpy.ok():
        rclpy.spin_once(a.n, timeout_sec=0.02)
        if not announced and (a.c_get.service_is_ready() or time.monotonic() - t_ready > 5.0):
            announced = True
            a.get(0)
        if not a.step():
            break
    a.n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
