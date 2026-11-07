#!/usr/bin/env python3
"""Live check of an exported .xosc: run ScenarioRunner on it and watch the world from a 2nd client.

    python tools/sr_live_check.py --xosc <dir>/variants/scenario_00_ego_5105.xosc --port 4100

Needs a running CARLA 0.9.16 server with the file's map. Starts ScenarioRunner
(`scenario_runner.py --openscenario <xosc> --sync --timeout 300 --port P`) as a child and, while
it runs, records every vehicle's transform per frame by role_name. Reports:
  * entities in the file vs actors that ever appeared -> spawn failures (ScenarioRunner only
    prints "WARNING: Not all actors were spawned" and carries on)
  * per entity, the worst distance between the actor's reference point (the hero's bounding-box
    centre, every other vehicle's origin) and the trajectory at that simulation time (t0 fitted
    on the hero)
  * frames, wall time, ScenarioRunner's exit code and the tail of its output
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import carla


def trajectories(xosc: Path) -> dict:
    """{entity: (times, [(x, y, yaw_deg)] in the CARLA frame)} from the FollowTrajectoryActions."""
    root = ET.parse(xosc).getroot()
    out = {}
    for mg in root.iter("ManeuverGroup"):
        name = mg.find("Actors/EntityRef").get("entityRef")
        times, pts = [], []
        for v in mg.iter("Vertex"):
            wp = v.find("Position/WorldPosition")
            times.append(float(v.get("time")))
            pts.append((float(wp.get("x")), -float(wp.get("y")),
                        -math.degrees(float(wp.get("h")))))
        if times:
            out[name] = (times, pts)
    names = [so.get("name") for so in root.iter("ScenarioObject")]
    return out, names


def at(traj, t):
    times, pts = traj
    if t <= times[0]:
        return pts[0]
    if t >= times[-1]:
        return pts[-1]
    i = bisect.bisect_right(times, t) - 1
    w = (t - times[i]) / (times[i + 1] - times[i])
    a, b = pts[i], pts[i + 1]
    return (a[0] + (b[0] - a[0]) * w, a[1] + (b[1] - a[1]) * w)


class Recorder:
    def __init__(self, client):
        self.world = client.get_world()
        self.frames = []            # (elapsed_s, {role: (cx, cy)})
        self.roles = {}             # actor id -> role_name
        self.boxes = {}             # actor id -> (bb.x, bb.y)
        self.stop = False

    def run(self):
        last_refresh = -1.0
        while not self.stop:
            try:
                snap = self.world.wait_for_tick(2.0)
            except RuntimeError:
                continue
            t = snap.timestamp.elapsed_seconds
            if t - last_refresh > 0.5:
                for a in self.world.get_actors().filter("vehicle.*"):
                    if a.id not in self.roles:
                        role = a.attributes.get("role_name", "")
                        self.roles[a.id] = role
                        bb = a.bounding_box
                        # the replay's reference point: the hero's box centre, any other origin
                        self.boxes[a.id] = ((bb.location.x, bb.location.y) if role == "hero"
                                            else (0.0, 0.0))
                last_refresh = t
            poses = {}
            for aid, role in self.roles.items():
                s = snap.find(aid)
                if s is None:
                    continue
                tf = s.get_transform()
                psi = math.radians(tf.rotation.yaw)
                ox, oy = self.boxes[aid]
                poses[role] = (tf.location.x + ox * math.cos(psi) - oy * math.sin(psi),
                               tf.location.y + ox * math.sin(psi) + oy * math.cos(psi))
            self.frames.append((t, poses))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--xosc", type=Path, required=True)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--sr", type=Path, default=Path(os.environ.get(
        "SCENARIO_RUNNER_ROOT", "~/scenario_runner")).expanduser())
    ap.add_argument("--sr-python", default=sys.executable)
    ap.add_argument("--extra", default="", help="more scenario_runner.py arguments")
    args = ap.parse_args()

    trajs, names = trajectories(args.xosc)
    client = carla.Client(args.host, args.port)
    client.set_timeout(300.0)
    rec = Recorder(client)
    th = threading.Thread(target=rec.run, daemon=True)
    th.start()
    cmd = [args.sr_python, str(args.sr / "scenario_runner.py"), "--openscenario",
           str(args.xosc.resolve()), "--sync", "--frameRate", "10", "--timeout", "300",
           "--host", args.host,
           "--port", str(args.port)] + args.extra.split()
    t_wall = time.time()
    p = subprocess.run(cmd, cwd=args.sr, capture_output=True, text=True,
                       env=dict(os.environ, PYTHONUNBUFFERED="1"))
    wall = time.time() - t_wall
    rec.stop = True
    th.join(5.0)

    seen = {r for _t, poses in rec.frames for r in poses}
    missing = sorted(set(names) - seen)
    # t0: the elapsed time of ScenarioRunner's time 0, fitted on the hero's first 3 s
    hero = [(t, poses["hero"]) for t, poses in rec.frames if "hero" in poses]
    best = None
    if hero and "hero" in trajs:
        for k in range(-40, 41):
            t0 = hero[0][0] + k * 0.025
            errs = [math.dist(p, at(trajs["hero"], t - t0)) for t, p in hero if t - t0 <= 3.0]
            if errs:
                e = max(errs)
                if best is None or e < best[1]:
                    best = (t0, e)
    worst = {}
    if best:
        t0 = best[0]
        for t, poses in rec.frames:
            for role, p in poses.items():
                if role in trajs and 0.0 <= t - t0 <= trajs[role][0][-1]:
                    worst[role] = max(worst.get(role, 0.0), math.dist(p, at(trajs[role], t - t0)))
    devs = sorted(worst.values())
    report = {
        "file": args.xosc.name, "sr_rc": p.returncode, "wall_s": round(wall, 1),
        "frames_seen": len(rec.frames), "entities_in_file": len(names),
        "entities_seen": len(seen & set(names)), "missing": missing[:20],
        "n_missing": len(missing), "t0_fit_err_m": round(best[1], 4) if best else None,
        "dev_m_median": round(devs[len(devs) // 2], 4) if devs else None,
        "dev_m_p99": round(devs[int(0.99 * (len(devs) - 1))], 4) if devs else None,
        "dev_m_max": round(devs[-1], 4) if devs else None,
        "worst_entities": sorted(worst, key=worst.get, reverse=True)[:5],
        "sr_warnings": [l for l in (p.stdout + p.stderr).splitlines()
                        if "WARNING" in l or "Error" in l][:20],
        "sr_tail": (p.stdout + p.stderr).splitlines()[-15:],
    }
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
