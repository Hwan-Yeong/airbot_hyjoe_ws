#!/usr/bin/env python3
"""follow-me 조정 화면 서버 (PC 에서 실행, 표준 라이브러리만 씀). 브라우저: http://localhost:8765

tune_agent.py 를 --agent-cmd 로 띄우고(로컬이면 python3, 로봇이면 SSH 명령) stdin/stdout 으로 잇는다.
- 파라미터 읽기·적용(한꺼번에), START/STOP/비상정지, 실시간 상태·그래프
- 프리셋 저장·불러오기, 변경 이력(CSV), 현재 값을 YAML 로 내보내기 (data-dir 아래)
- --hook "이름=셸 명령" 을 주면 화면에 버튼이 생긴다 (예: 컨테이너 시작·정지)
usage: tune_server.py [--agent-cmd CMD] [--port 8765] [--data-dir ~/.kebi_follow_tune]
                      [--defaults-yaml follow_controller.yaml] [--hook NAME=CMD ...]
"""
import argparse
import collections
import csv
import datetime
import http.server
import json
import os
import shlex
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TEL_KEEP = 600            # 그래프용 상태 보관 개수 (10 Hz → 60 s)
LOG_KEEP = 200
REPLY_TIMEOUT_S = 8.0


def load_defaults(path):
    """follow_controller.yaml 의 실수 파라미터 (기본값 표시용). PyYAML 이 없으면 빈 값."""
    if not path or not os.path.exists(path):
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    p = (doc.get("follow_controller") or {}).get("ros__parameters") or {}
    return {k: float(v) for k, v in p.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}


class AgentLink:
    def __init__(self, cmd, log):
        self.cmd = cmd
        self.log = log
        self.lock = threading.Lock()
        self.proc = None
        self.next_id = 1
        self.replies = {}
        self.cv = threading.Condition(self.lock)
        self.tel = collections.deque(maxlen=TEL_KEEP)
        self.tel_seq = 0
        self.params, self.ranges = {}, {}
        self.connected = False
        self.started_at = 0.0
        self.start()

    def start(self):
        with self.lock:
            if self.proc and self.proc.poll() is None:
                self.proc.kill()
            self.connected = False
            self.started_at = time.time()
            self.proc = subprocess.Popen(["bash", "-c", self.cmd], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=subprocess.PIPE, text=True, bufsize=1)
        threading.Thread(target=self._read, args=(self.proc,), daemon=True).start()
        threading.Thread(target=self._err, args=(self.proc,), daemon=True).start()
        self.log("agent", "중계 시작: " + self.cmd[:120])

    def _err(self, proc):
        for line in proc.stderr:
            line = line.strip()
            if line and "no job control" not in line and "cannot set terminal process group" not in line:
                self.log("agent", line[:300])

    def _read(self, proc):
        for line in proc.stdout:
            line = line.strip().strip("\r")
            if not line.startswith("{"):
                continue
            try:
                m = json.loads(line)
            except json.JSONDecodeError:
                continue
            with self.lock:
                kind = m.get("type")
                if kind == "hello":
                    self.connected = True
                elif kind == "tel":
                    self.tel_seq += 1
                    m["seq"] = self.tel_seq
                    self.tel.append(m)
                elif kind == "params":
                    self.params, self.ranges = m.get("values", {}), m.get("ranges", {})
                elif kind == "result":
                    self.replies[m.get("id")] = m
                    self.cv.notify_all()
                elif kind == "log":
                    self.log("agent", m.get("msg", ""))
        with self.lock:
            if proc is self.proc:
                self.connected = False
        self.log("agent", "중계 끊김")

    def request(self, op, timeout=REPLY_TIMEOUT_S, **kw):
        with self.lock:
            if not self.proc or self.proc.poll() is not None:
                return {"ok": False, "reason": "중계가 실행 중이 아님 (다시 연결 눌러 주세요)"}
            rid = self.next_id
            self.next_id += 1
            try:
                self.proc.stdin.write(json.dumps(dict(op=op, id=rid, **kw)) + "\n")
                self.proc.stdin.flush()
            except (BrokenPipeError, OSError) as e:
                return {"ok": False, "reason": f"중계로 보내지 못함: {e}"}
            end = time.time() + timeout
            while rid not in self.replies:
                left = end - time.time()
                if left <= 0:
                    return {"ok": False, "reason": "응답 시간 초과"}
                self.cv.wait(left)
            return self.replies.pop(rid)


class App:
    def __init__(self, a):
        self.a = a
        os.makedirs(a.data_dir, exist_ok=True)
        self.preset_path = os.path.join(a.data_dir, "presets.json")
        self.csv_path = os.path.join(a.data_dir, "tune_log.csv")
        self.logs = collections.deque(maxlen=LOG_KEEP)
        self.loglock = threading.Lock()
        self.defaults = load_defaults(a.defaults_yaml)
        self.hooks = dict(h.split("=", 1) for h in a.hook)
        self.agent = AgentLink(a.agent_cmd, self.log)

    def log(self, kind, msg):
        with self.loglock:
            self.logs.append({"t": time.time(), "kind": kind, "msg": msg})
        print(f"[{kind}] {msg}", file=sys.stderr, flush=True)

    def presets(self):
        try:
            with open(self.preset_path) as f:
                return json.load(f)
        except (OSError, json.JSONDecodeError):
            return {}

    def save_presets(self, p):
        with open(self.preset_path, "w") as f:
            json.dump(p, f, ensure_ascii=False, indent=1)

    def state(self, since):
        ag = self.agent
        with ag.lock:
            tel = [m for m in ag.tel if m["seq"] > since]
            st = {"connected": ag.connected and ag.proc is not None and ag.proc.poll() is None,
                  "params": ag.params, "ranges": ag.ranges, "tel": tel[-TEL_KEEP:]}
        with self.loglock:
            st["log"] = list(self.logs)[-60:]
        st["defaults"] = self.defaults
        st["presets"] = self.presets()
        st["hooks"] = list(self.hooks)
        return st

    def set_values(self, values):
        old = dict(self.agent.params)
        r = self.agent.request("set", values=values)
        stamp = datetime.datetime.now().isoformat(timespec="seconds")
        new_file = not os.path.exists(self.csv_path)
        with open(self.csv_path, "a", newline="") as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(["time", "param", "old", "new", "ok", "reason"])
            for k, v in values.items():
                w.writerow([stamp, k, old.get(k, ""), v, r.get("ok"), r.get("reason", "")])
        txt = ", ".join(f"{k} {old.get(k, '?')} → {v}" for k, v in values.items())
        self.log("set", ("적용: " if r.get("ok") else "거부: ") + txt + ("" if r.get("ok") else f" ({r.get('reason')})"))
        return r

    def export_yaml(self):
        vals = self.agent.params
        lines = ["# kebi follow-me 조정 결과 (" + datetime.datetime.now().isoformat(timespec="seconds") + ")",
                 "# 로봇 덮어쓰기 파일(params:=...)에 붙여 쓰면 다음 실행부터 기본값이 된다.",
                 "follow_controller:", "  ros__parameters:"]
        lines += [f"    {k}: {v:.4g}" for k, v in sorted(vals.items())]
        text = "\n".join(lines) + "\n"
        d = os.path.join(self.a.data_dir, "exports")
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, datetime.datetime.now().strftime("follow_tuned_%Y%m%d_%H%M%S.yaml"))
        with open(path, "w") as f:
            f.write(text)
        self.log("export", path)
        return {"ok": True, "path": path, "text": text}

    def run_hook(self, name):
        cmd = self.hooks.get(name)
        if cmd is None:
            return {"ok": False, "reason": f"없는 버튼 {name}"}
        self.log("hook", f"{name}: {cmd}")
        try:
            p = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=60)
        except subprocess.TimeoutExpired:
            return {"ok": False, "reason": "60 s 안에 끝나지 않음"}
        tail = (p.stdout + p.stderr).strip().splitlines()[-6:]
        self.log("hook", f"{name} 끝 (rc={p.returncode}) " + " | ".join(tail)[:300])
        return {"ok": p.returncode == 0, "reason": " | ".join(tail)}


def make_handler(app):
    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = open(os.path.join(HERE, "index.html"), "rb").read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path.startswith("/api/state"):
                since = 0
                if "since=" in self.path:
                    try:
                        since = int(self.path.split("since=")[1].split("&")[0])
                    except ValueError:
                        since = 0
                self._json(app.state(since))
            else:
                self._json({"ok": False, "reason": "not found"}, 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length", "0") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                body = {}
            p = self.path
            if p == "/api/set":
                vals = {k: float(v) for k, v in (body.get("values") or {}).items()}
                self._json(app.set_values(vals) if vals else {"ok": False, "reason": "바꾼 값 없음"})
            elif p == "/api/get":
                self._json(app.agent.request("get"))
            elif p == "/api/cmd":
                c = body.get("cmd", "")
                r = app.agent.request("cmd", cmd=c)
                app.log("cmd", f"{c}: {'확인' if r.get('ok') else '실패 ' + r.get('reason', '')}")
                self._json(r)
            elif p == "/api/preset/save":
                name = (body.get("name") or "").strip()
                if not name:
                    self._json({"ok": False, "reason": "이름이 비었음"})
                    return
                ps = app.presets()
                ps[name] = {"values": body.get("values") or app.agent.params, "saved": time.time()}
                app.save_presets(ps)
                app.log("preset", f"저장: {name}")
                self._json({"ok": True})
            elif p == "/api/preset/delete":
                ps = app.presets()
                ps.pop(body.get("name", ""), None)
                app.save_presets(ps)
                self._json({"ok": True})
            elif p == "/api/export":
                self._json(app.export_yaml())
            elif p == "/api/agent/restart":
                app.agent.start()
                self._json({"ok": True})
            elif p == "/api/hook":
                self._json(app.run_hook(body.get("name", "")))
            else:
                self._json({"ok": False, "reason": "not found"}, 404)

    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--agent-cmd", default=f"exec python3 -u {shlex.quote(os.path.join(HERE, 'tune_agent.py'))}")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--data-dir", default=os.path.expanduser("~/.kebi_follow_tune"))
    ap.add_argument("--defaults-yaml", default=os.path.join(HERE, "..", "..", "..", "kebi_follow_controller",
                                                            "config", "follow_controller.yaml"))
    ap.add_argument("--hook", action="append", default=[], help='"버튼 이름=셸 명령"')
    a = ap.parse_args()
    app = App(a)
    srv = http.server.ThreadingHTTPServer((a.bind, a.port), make_handler(app))
    app.log("server", f"http://localhost:{a.port} (data {a.data_dir})")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if app.agent.proc and app.agent.proc.poll() is None:
            app.agent.proc.kill()


if __name__ == "__main__":
    main()
