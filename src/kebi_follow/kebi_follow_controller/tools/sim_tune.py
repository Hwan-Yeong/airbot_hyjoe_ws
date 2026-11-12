#!/usr/bin/env python3
"""follow_core_sim 으로 부드러움 튜닝 (격자 탐색 + 점수). 표준 라이브러리 + PyYAML.

점수(낮을수록 좋음) = 시나리오 합:
  RMS jerk + 0.25·각 RMS jerk + 0.15·이동 중 최대 jerk + 0.03·최대 각 jerk
  + 1.5·출발-정지 반복 + 2·단호 정지 + 추종 거리 벌점(1.1~1.6 m 밖) + 안전 벌점(사람 0.75 m, 벽 0.2 m)
usage:
  sim_tune.py --sim BIN [--yaml follow_controller.yaml] [--set k=v ...] [--seeds 3]           # 한 설정 평가
  sim_tune.py --sim BIN ... --grid k=v1,v2 --grid k2=... [--top 10]                             # 격자 탐색
"""
import argparse
import itertools
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WALKING = {"walk_stop_walk", "slow_irregular", "turn_left_90", "occlusion_resume", "ambiguous_blip", "wall_ahead"}


def tunable_names():
    hdr = os.path.join(HERE, "..", "include", "kebi_follow_controller", "tunables.hpp")
    return re.findall(r'\{"([a-z_]+)",\s*[-0-9.]+,', open(hdr).read())


def yaml_params(path):
    import yaml
    p = yaml.safe_load(open(path))["follow_controller"]["ros__parameters"]
    names = set(tunable_names())
    return {k: float(v) for k, v in p.items() if k in names and isinstance(v, (int, float)) and not isinstance(v, bool)}


def run(sim, params, seed, out=None, scenario=None):
    cmd = [sim, "--quiet", "--seed", str(seed)]
    for k, v in params.items():
        cmd += ["--set", f"{k}={v}"]
    if out:
        os.makedirs(out, exist_ok=True)
        cmd += ["--out", out]
    if scenario:
        cmd += ["--scenario", scenario]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr.strip())
    return json.loads(r.stdout)["results"]


def score_one(m):
    s = m["rms_jerk_cmd"] + 0.25 * m["rms_wjerk_cmd"] + 0.15 * m["max_jerk_moving"] + 0.03 * m["max_wjerk_cmd"]
    s += 1.5 * m["stop_go"] + 2.0 * m["firm"]
    if m["scenario"] in WALKING:
        s += 2.0 * max(0.0, m["mean_dist"] - 1.6) + 2.0 * max(0.0, 1.1 - m["mean_dist"])
    if m["scenario"] != "approach_back" and m["min_dist"] < 0.75:
        s += 20.0
    if m["min_wall"] < 0.2:
        s += 20.0
    if m["scenario"] == "fast_walk":
        s += 1.0 * max(0.0, m["mean_dist"] - 2.6)
    # 반응성: 사람이 출발한 뒤 0.8 s 넘게 서 있거나, 사람 쪽 방향 오차가 30° 를 자주 넘으면 벌점
    s += 2.0 * max(0.0, m["start_lag"] - 0.8)
    if m["scenario"] not in ("side_step", "circle_around"):
        s += 0.05 * max(0.0, m["bearing_p95"] - 30.0)
    return s


def evaluate(sim, params, seeds):
    per = {}
    for sd in range(1, seeds + 1):
        for m in run(sim, params, sd):
            per.setdefault(m["scenario"], []).append(m)
    total, rows = 0.0, {}
    for name, ms in per.items():
        avg = {k: sum(x[k] for x in ms) / len(ms) for k in ms[0] if k != "scenario"}
        avg["scenario"] = name
        avg["score"] = sum(score_one(x) for x in ms) / len(ms)
        total += avg["score"]
        rows[name] = avg
    return total, rows


def table(rows):
    keys = ["score", "stop_go", "firm", "rms_jerk_cmd", "max_jerk_moving", "rms_wjerk_cmd", "max_wjerk_cmd", "min_dist", "mean_dist", "start_lag", "bearing_p95"]
    out = ["scenario".ljust(18) + "".join(k[:10].rjust(11) for k in keys)]
    for n, r in rows.items():
        out.append(n.ljust(18) + "".join(f"{r[k]:11.2f}" for k in keys))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", required=True)
    ap.add_argument("--yaml", default=os.path.join(HERE, "..", "config", "follow_controller.yaml"))
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--grid", action="append", default=[])
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    base = yaml_params(a.yaml)
    for kv in a.set:
        k, v = kv.split("=", 1)
        base[k] = float(v)
    if not a.grid:
        total, rows = evaluate(a.sim, base, a.seeds)
        print(table(rows))
        print(f"TOTAL {total:.2f}")
        if a.json:
            json.dump({"params": base, "total": total, "rows": rows}, open(a.json, "w"), indent=1)
        return
    axes = []
    for g in a.grid:
        k, vs = g.split("=", 1)
        axes.append((k, [float(x) for x in vs.split(",")]))
    results = []
    combos = list(itertools.product(*[v for _, v in axes]))
    for i, combo in enumerate(combos):
        p = dict(base)
        p.update({k: v for (k, _), v in zip(axes, combo)})
        try:
            total, _ = evaluate(a.sim, p, a.seeds)
        except RuntimeError as e:
            continue
        results.append((total, dict(zip([k for k, _ in axes], combo))))
        if (i + 1) % 200 == 0 and results:   # 점수가 같으면 dict 끼리 비교하지 않게 점수로만
            print(f"  {i + 1}/{len(combos)} best {min(r[0] for r in results):.2f}", file=sys.stderr, flush=True)
    results.sort(key=lambda r: r[0])
    for total, combo in results[: a.top]:
        print(f"{total:8.2f}  " + "  ".join(f"{k}={v:g}" for k, v in combo.items()))
    if a.json:
        json.dump([{"total": t, "params": c} for t, c in results[:50]], open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
