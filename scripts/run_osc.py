#!/usr/bin/env python3
"""Run one munich2drive OpenSCENARIO file in CARLA with ScenarioRunner 0.9.16 and record the
ego's camera views plus an edge-detection (Canny) video of every view.

    python scripts/run_osc.py path/to/scenario_00_ego_5105.xosc --port 4100 --out runs/

ScenarioRunner itself is not modified; this script patches it in-process:
  1. weather   The scenario's exact CARLA weather (its Carla_* parameters) is set before the
               actors spawn and again where ScenarioRunner applies the Environment, which it
               rebuilds from CARLA defaults (a cloudy clip would get a black sky on these maps).
  2. time      OpenSCENARIO's SimulationTimeCondition is absolute simulation time; ScenarioRunner
               starts its clock when the event starts, which makes every DeleteEntityAction fire
               0.2 s late. The conditions count from t = 0 here (TimeOut keeps its own clock).
  3. spawning  ScenarioRunner batch-spawns every agent 0.2 m above its pose and only warns when
               CARLA refuses one (its body touches the road or another car). A refused agent is
               spawned again with more lift, then high in the air. An agent beyond UE4's 10.48 km
               physics bound (Munich's east end) is spawned inside it and moved out, or CARLA
               reports a 0 x 0 x 0 bounding box for it.
  4. planner   ScenarioRunner builds CARLA's GlobalRoutePlanner for every map. On Belmont, 3 road
               segments (road 137, lane 2) never reach their end by `next()` and the planner's
               loop grows until the process is killed; every segment's walk is capped here.
  5. ticks     Every actor's pose comes from osc_replay_control's placement (the controller the
               files assign, shipped in osc/): before each world tick all poses for that tick go
               out in ONE synchronous batch (a controller's own set_transform is asynchronous and
               landed after the tick once in ~37,000 actor-frames). Before the first tick every
               actor is placed, the Munich lamp heads go dark (--lamp-bridge) and the cameras are
               attached; one pre-roll tick films t = 0. After each tick the script waits for every
               view's image of exactly that frame, pipes it into one ffmpeg per view (<view>.mp4 +
               <view>_edge.mp4), and audits every actor against the file.
ScenarioRunner runs with --sync --frameRate 10: one tick per scenario step, 200 frames.
Output: <out>/<clip>/{<view>.mp4, <view>_edge.mp4, run.json, scenario_runner.log}.
Exit status 0 only when the run completed with every agent and every frame.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
VIEWS = ("front", "front_left", "front_right", "back", "back_left", "back_right", "bird", "chase")
PHYSICS_BOUND_M = 10400.0         # UE4 destroys a physics body beyond 10,485.76 m on any axis
DT = 0.1


# ------------------------------------------------------------------ the file
def parse_xosc(path: Path) -> dict:
    """Entities, Init poses (CARLA frame) and parameters of a scenario file."""
    root = ET.parse(path).getroot()
    params = {p.get("name"): p.get("value") for p in root.iter("ParameterDeclaration")}
    ents = {}
    for so in root.iter("ScenarioObject"):
        veh = so.find("Vehicle")
        props = {p.get("name"): p.get("value") for p in veh.iter("Property")} if veh is not None else {}
        ents[so.get("name")] = {"model": veh.get("name") if veh is not None else None,
                                "color": props.get("color")}
    init = {}
    for pr in root.find("Storyboard/Init/Actions").iter("Private"):
        wp = pr.find(".//TeleportAction/Position/WorldPosition")
        if wp is not None:
            init[pr.get("entityRef")] = (float(wp.get("x")), -float(wp.get("y")))
    return {"params": params, "entities": ents, "init": init,
            "map": root.find("RoadNetwork/LogicFile").get("filepath")}


def interp_xy(traj, t: float):
    """(x, y) of a file trajectory at time t, held at the ends."""
    ts, pts = traj["t"], traj["pts"]
    if t <= ts[0]:
        return pts[0][0], pts[0][1]
    if t >= ts[-1]:
        return pts[-1][0], pts[-1][1]
    import bisect
    i = bisect.bisect_right(ts, t) - 1
    w = (t - ts[i]) / (ts[i + 1] - ts[i])
    return (pts[i][0] + (pts[i + 1][0] - pts[i][0]) * w,
            pts[i][1] + (pts[i + 1][1] - pts[i][1]) * w)


# ------------------------------------------------------------------ video
class ViewWriter:
    """One ffmpeg per view: raw BGRA frames in; <view>.mp4 and <view>_edge.mp4 out. The edge
    video is computed from the UNCOMPRESSED frame (gray -> Canny), so it holds no codec edges."""

    def __init__(self, out_dir: Path, view: str, w: int, h: int, fps: int, crop,
                 low: float, high: float, crf: int):
        chain = f"crop={crop[2]}:{crop[3]}:{crop[0]}:{crop[1]}," if crop else ""
        graph = (f"[0:v]{chain}split=2[rgb][e0];"
                 f"[e0]format=gray,edgedetect=low={low}:high={high}[edge]")
        enc = ["-c:v", "libx264", "-preset", "medium", "-crf", str(crf), "-pix_fmt", "yuv420p"]
        self.paths = (out_dir / f"{view}.mp4", out_dir / f"{view}_edge.mp4")
        cmd = ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgra",
               "-s", f"{w}x{h}", "-r", str(fps), "-i", "-", "-filter_complex", graph,
               "-map", "[rgb]", *enc, str(self.paths[0]), "-map", "[edge]", *enc,
               str(self.paths[1])]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self.frames = 0

    def put(self, raw) -> None:
        self.proc.stdin.write(raw)
        self.frames += 1

    def close(self) -> int:
        try:
            self.proc.stdin.close()
        except BrokenPipeError:
            pass
        return self.proc.wait()


class Rig:
    """The clips' camera rig on the hero; collect(frame) blocks until every view has it.
    A failure while building it destroys what was already spawned and started."""

    def __init__(self, carla, world, hero, rig: dict, views, out_dir: Path, fps: int,
                 edge_low: float, edge_high: float, crf: int):
        self.cams, self.queues, self.writers = {}, {}, {}
        self.missed = {v: 0 for v in views}
        try:
            lib = world.get_blueprint_library()
            for v in views:
                spec = rig["views"][v]
                bp = lib.find("sensor.camera.rgb")
                bp.set_attribute("image_size_x", str(spec["render_width"]))
                bp.set_attribute("image_size_y", str(spec["render_height"]))
                bp.set_attribute("fov", str(spec["render_fov"]))
                if bp.has_attribute("gamma"):
                    bp.set_attribute("gamma", str(rig.get("gamma", 2.2)))
                tf = carla.Transform(carla.Location(x=spec["x"], y=spec["y"], z=spec["z"]),
                                     carla.Rotation(pitch=spec["pitch"], yaw=spec["yaw"],
                                                    roll=spec["roll"]))
                self.writers[v] = ViewWriter(out_dir, v, spec["render_width"],
                                             spec["render_height"], fps, spec.get("crop"),
                                             edge_low, edge_high, crf)
                cam = world.spawn_actor(bp, tf, attach_to=hero,
                                        attachment_type=carla.AttachmentType.Rigid)
                q: queue.Queue = queue.Queue()
                cam.listen(q.put)
                self.cams[v], self.queues[v] = cam, q
        except BaseException:
            self.close()
            raise

    def collect(self, frame: int, timeout: float = 60.0) -> None:
        """Every view's image of `frame`, in order. Images of earlier frames are skipped; in
        synchronous mode a later frame cannot exist yet, so a missing image is an error."""
        for v, q in self.queues.items():
            while True:
                img = q.get(timeout=timeout)
                if img.frame == frame:
                    self.writers[v].put(img.raw_data)
                    break
                if img.frame > frame:
                    self.missed[v] += 1
                    self.writers[v].put(img.raw_data)
                    break

    def close(self) -> dict:
        for cam in self.cams.values():
            try:
                cam.stop()
                cam.destroy()
            except RuntimeError:
                pass
        out = {v: {"frames": w.frames, "missed": self.missed.get(v, 0), "ffmpeg_rc": w.close(),
                   "files": [p.name for p in w.paths]} for v, w in self.writers.items()}
        self.cams, self.writers = {}, {}
        return out


# ------------------------------------------------------------------ audit
class Audit:
    """Every captured frame: each actor's reference point (the hero's box centre, every other
    vehicle's origin) against the file's trajectory, and its height against the placement."""

    def __init__(self, trajs: dict, dt: float):
        self.trajs, self.dt = trajs, dt
        self.worst: dict = {}
        self.per_frame: list = []
        self.n = 0
        self.ghosts = 0
        self.max_dz = 0.0

    def frame(self, snapshot, k: int, ids: dict, boxes: dict, tracks: dict, pose_at) -> None:
        t = k * self.dt
        fmax = 0.0
        for role, aid in ids.items():
            traj = self.trajs.get(role)
            s = snapshot.find(aid)
            if traj is None or s is None:
                continue
            tf = s.get_transform()
            if traj["gone"] is not None and t > traj["gone"]:
                if tf.location.z > -500.0:             # still in view after its removal
                    self.ghosts += 1
                continue
            psi = math.radians(tf.rotation.yaw)
            ox, oy, _z = boxes[role]
            cx = tf.location.x + ox * math.cos(psi) - oy * math.sin(psi)
            cy = tf.location.y + ox * math.sin(psi) + oy * math.cos(psi)
            ex, ey = interp_xy(traj, t)
            d = math.hypot(cx - ex, cy - ey)
            want = pose_at(*tracks[role], t)
            self.max_dz = max(self.max_dz, abs(tf.location.z - want.location.z))
            if d > self.worst.get(role, (0.0, 0))[0]:
                self.worst[role] = (d, k)
            fmax = max(fmax, d)
            self.n += 1
        self.per_frame.append((k, round(fmax, 4)))

    def report(self) -> dict:
        devs = sorted(v[0] for v in self.worst.values())
        top = sorted(self.worst.items(), key=lambda kv: -kv[1][0])[:5]
        return {"actor_frames": self.n, "actors": len(self.worst),
                "max_m": round(devs[-1], 4) if devs else None,
                "p99_m": round(devs[int(0.99 * (len(devs) - 1))], 4) if devs else None,
                "median_m": round(devs[len(devs) // 2], 4) if devs else None,
                "max_dz_m": round(self.max_dz, 4),
                "worst": [{"actor": r, "dev_m": round(d, 4), "frame": k} for r, (d, k) in top],
                "frames_over_5cm": [(k, d) for k, d in self.per_frame if d > 0.05][:30],
                "in_view_after_delete": self.ghosts}


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("xosc", type=Path)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=4100)
    ap.add_argument("--tm-port", type=int, default=8100, help="Traffic Manager port")
    ap.add_argument("--out", type=Path, default=Path("runs"))
    ap.add_argument("--sr-root", type=Path, default=Path(os.environ.get(
        "SCENARIO_RUNNER_ROOT", "~/scenario_runner")).expanduser())
    ap.add_argument("--carla-pythonapi", type=Path, default=None,
                    help="CARLA's PythonAPI/carla folder (ScenarioRunner imports its `agents`); "
                         "default $CARLA_ROOT/PythonAPI/carla")
    ap.add_argument("--rig", type=Path, default=REPO / "rigs" / "edgar_rig.json")
    ap.add_argument("--views", default=",".join(VIEWS))
    ap.add_argument("--no-record", action="store_true", help="run without cameras")
    ap.add_argument("--lamp-bridge", default=None,
                    help="Munich server: the -GermanLightControl prefix it was started with")
    ap.add_argument("--lamp-heads", type=Path, default=None,
                    help="german_light_heads.json (download_server.py puts it in the Munich "
                         "server folder); needed with --lamp-bridge")
    ap.add_argument("--edge-low", type=float, default=0.08)
    ap.add_argument("--edge-high", type=float, default=0.25)
    ap.add_argument("--crf", type=int, default=18)
    ap.add_argument("--timeout", type=float, default=300.0)
    args = ap.parse_args()

    if args.lamp_bridge and not args.lamp_heads:
        ap.error("--lamp-bridge needs --lamp-heads <server>/german_light_heads.json")
    rig_spec = None if args.no_record else json.loads(args.rig.read_text())
    views = [v for v in args.views.split(",") if v]
    if rig_spec is not None:
        bad = [v for v in views if v not in rig_spec["views"]]
        if bad:
            ap.error(f"--views {bad} not in {args.rig} ({sorted(rig_spec['views'])})")
    xosc = args.xosc.resolve()
    X = parse_xosc(xosc)
    clip = xosc.stem
    out_dir = (args.out / clip).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    capi = args.carla_pythonapi or (Path(os.environ["CARLA_ROOT"]) / "PythonAPI" / "carla"
                                    if os.environ.get("CARLA_ROOT") else None)
    # the runner's controller first: ScenarioRunner then imports this very module object
    paths = [str(REPO / "osc"), str(args.sr_root.resolve()), str(xosc.parent)]
    if capi:
        paths.append(str(Path(capi).resolve()))
    sys.path[:0] = paths

    import carla
    import numpy as np
    import py_trees
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
    from srunner.scenariomanager.scenario_manager import ScenarioManager
    from srunner.scenariomanager.timer import GameTime, SimulationTimeCondition
    from srunner.scenarios.open_scenario import OpenScenario
    from srunner.tools.openscenario_parser import OpenScenarioParser
    import scenario_runner as sr_main
    import osc_replay_control as orc
    orc.EXTERNAL_PLACEMENT = True            # every pose comes from the batch below

    trajs = orc.read_file(str(xosc))
    horizon = max((len(tr["t"]) for tr in trajs.values()), default=0)
    exact_weather = {k[len("Carla_"):]: float(v) for k, v in X["params"].items()
                     if k.startswith("Carla_")}
    STATE: dict = {"spawn": {"stock_refused": [], "respawned": {}, "lost": [],
                             "beyond_bound": []},
                   "weather": exact_weather or None, "lamps": None, "frames": 0, "colors": {},
                   "planner_capped": 0, "capture_error": None, "sr_returned": False,
                   "batch_errors": 0, "views": None}
    holder: dict = {"rig": None, "ids": {}, "boxes": {}, "tracks": {}, "k": 0, "tree": None}
    audit = Audit(trajs, DT)

    # 1. weather: exact, before the actors spawn and where ScenarioRunner sets the Environment
    def initialize_environment(self, world):
        if exact_weather:
            world.set_weather(carla.WeatherParameters(**exact_weather))
    OpenScenario._initialize_environment = initialize_environment
    _orig_weather = OpenScenarioParser.get_weather_from_env_action

    def get_weather(xml_tree, catalogs):
        w = _orig_weather(xml_tree, catalogs)
        if exact_weather:
            w.carla_weather = carla.WeatherParameters(**exact_weather)
        return w
    OpenScenarioParser.get_weather_from_env_action = staticmethod(get_weather)

    # 2. time: SimulationTimeCondition counts from t = 0 (OpenSCENARIO's absolute time); its
    #    subclass TimeOut, used for Condition@delay and the scenario timeout, stays relative
    _orig_init = SimulationTimeCondition.initialise

    def stc_initialise(self):
        if type(self) is SimulationTimeCondition:
            self._start_time = 0.0
        else:
            _orig_init(self)
    SimulationTimeCondition.initialise = stc_initialise

    # 3. spawning
    _orig_one = CarlaDataProvider.request_new_actor
    _orig_many = CarlaDataProvider.request_new_actors

    def inside_bound(tf):
        x, y = tf.location.x, tf.location.y
        lim = PHYSICS_BOUND_M - 20.0
        if abs(x) <= PHYSICS_BOUND_M and abs(y) <= PHYSICS_BOUND_M:
            return tf, False
        return carla.Transform(carla.Location(x=max(-lim, min(lim, x)), y=max(-lim, min(lim, y)),
                                              z=tf.location.z + 2.0), tf.rotation), True

    def request_new_actor(model, spawn_point=None, rolename="scenario", autopilot=False,
                          random_location=False, color=None, actor_category="car",
                          attribute_filter=None, tick=True):
        if spawn_point is not None:             # the ego comes this way, not through the batch
            spawn_point, moved = inside_bound(spawn_point)
            if moved and rolename not in STATE["spawn"]["beyond_bound"]:
                STATE["spawn"]["beyond_bound"].append(rolename)
        a = _orig_one(model, spawn_point, rolename, autopilot, random_location, color,
                      actor_category, attribute_filter, tick)
        if a is not None or spawn_point is None or rolename not in X["init"]:
            return a
        for lift in (0.6, 1.2, 2.5, 300.0 + 6.0 * len(STATE["spawn"]["respawned"])):
            sp = carla.Transform(carla.Location(x=spawn_point.location.x,
                                                y=spawn_point.location.y,
                                                z=spawn_point.location.z + lift),
                                 spawn_point.rotation)
            a = _orig_one(model, sp, rolename, autopilot, False, color, actor_category,
                          attribute_filter, tick)
            if a is not None:
                if rolename in trajs:              # a replayed agent; an ego you drive keeps physics
                    a.set_simulate_physics(False)
                STATE["spawn"]["respawned"][rolename] = lift
                return a
        STATE["spawn"]["lost"].append(rolename)
        return None
    CarlaDataProvider.request_new_actor = staticmethod(request_new_actor)

    def request_new_actors(actor_list, attribute_filter=None, tick=True):
        for c in actor_list:                    # spawn inside the physics bound, move out later
            c.transform, moved = inside_bound(c.transform)
            if moved:
                STATE["spawn"]["beyond_bound"].append(c.rolename)
        actors = list(_orig_many(actor_list, attribute_filter, tick) or [])
        got = {a.attributes.get("role_name") for a in actors}
        for c in actor_list:
            if c.rolename in got:
                continue
            STATE["spawn"]["stock_refused"].append(c.rolename)
            a = request_new_actor(c.model, c.transform, c.rolename, color=c.color,
                                  actor_category=c.category, tick=False)
            if a is not None:
                actors.append(a)
        if STATE["spawn"]["stock_refused"]:
            CarlaDataProvider.get_world().tick()
        return actors or None
    CarlaDataProvider.request_new_actors = staticmethod(request_new_actors)

    # 4. planner: CARLA's GlobalRoutePlanner._build_topology with every segment's walk capped
    def build_topology_capped(self):
        self._topology = []
        res = self._sampling_resolution
        for wp1, wp2 in self._wmap.get_topology():
            l1, l2 = wp1.transform.location, wp2.transform.location
            x1, y1, z1, x2, y2, z2 = np.round([l1.x, l1.y, l1.z, l2.x, l2.y, l2.z], 0)
            wp1.transform.location, wp2.transform.location = l1, l2
            seg = {"entry": wp1, "exit": wp2, "entryxyz": (x1, y1, z1),
                   "exitxyz": (x2, y2, z2), "path": []}
            endloc = wp2.transform.location
            if wp1.transform.location.distance(endloc) > res:
                # generous: long curved segments (Munich's ramps) finish; only an endless walk stops
                cap = int(20 * wp1.transform.location.distance(endloc) / res) + 5000
                w = wp1.next(res)[0]
                while w.transform.location.distance(endloc) > res:
                    seg["path"].append(w)
                    if len(seg["path"]) > cap:      # never reaches its end: keep what we have
                        STATE["planner_capped"] += 1
                        break
                    nxt = w.next(res)
                    if not nxt:
                        break
                    w = nxt[0]
            else:
                nxt = wp1.next(res)
                if not nxt:
                    continue
                seg["path"].append(nxt[0])
            self._topology.append(seg)
    GlobalRoutePlanner._build_topology = build_topology_capped

    # 5. ticks
    def flush_poses(t: float) -> None:
        """Every scenario actor on its pose at time t, in one synchronous batch."""
        batch = []
        for role, aid in holder["ids"].items():
            tr = holder["tracks"].get(role)
            if tr is None:
                continue
            gone = trajs[role]["gone"]
            if gone is not None and t > gone:
                continue
            batch.append(carla.command.ApplyTransform(aid, orc.pose_at(*tr, t)))
        if batch:
            for r in CarlaDataProvider.get_client().apply_batch_sync(batch, False):
                if r.error:
                    STATE["batch_errors"] += 1

    def capture(world, frame) -> None:
        if holder["rig"] is not None:
            holder["rig"].collect(frame)
        audit.frame(world.get_snapshot(), holder["k"], holder["ids"], holder["boxes"],
                    holder["tracks"], orc.pose_at)
        holder["k"] += 1
        STATE["frames"] = holder["k"]
        if holder["k"] % 50 == 0:
            print(f"[run_osc] frame {holder['k']}/{horizon} at GameTime "
                  f"{GameTime.get_time():.2f} s", file=sys.__stderr__, flush=True)

    _orig_run = ScenarioManager.run_scenario

    def run_scenario(self):
        world = CarlaDataProvider.get_world()
        cmap = CarlaDataProvider.get_map()
        try:
            actors = [a for a in list(self.ego_vehicles) + list(self.other_actors) if a is not None]
            for a in actors:
                role = a.attributes.get("role_name")
                if role in trajs:                    # replayed; an ego under your control keeps physics
                    a.set_simulate_physics(False)
                holder["ids"][role] = a.id
                holder["boxes"][role] = orc.replay_box(a, role)
                if role in trajs:
                    holder["tracks"][role] = (trajs[role]["t"], orc.resolve_track(
                        cmap, trajs[role], holder["boxes"][role]))
                want = X["entities"].get(role, {}).get("color")
                if want:
                    STATE["colors"][role] = (a.attributes.get("color") == want)
            flush_poses(0.0)
            if args.lamp_bridge:
                STATE["lamps"] = lamps_dark(world, args.lamp_bridge, args.lamp_heads)
                flush_poses(0.0)
            hero = next((a for a in actors if a.attributes.get("role_name") == "hero"), None)
            if rig_spec is not None and hero is not None:
                holder["rig"] = Rig(carla, world, hero, rig_spec, views, out_dir,
                                    int(rig_spec.get("fps", 10)), args.edge_low,
                                    args.edge_high, args.crf)
            capture(world, world.tick())             # pre-roll: the frame of t = 0
            _orig_run(self)
        except Exception as e:                       # noqa: BLE001 -- reported in run.json
            STATE["capture_error"] = f"{type(e).__name__}: {e}"
            raise
        finally:
            if self._watchdog is not None:           # a non-daemon thread: never leave it running
                self._watchdog.stop()
                self._watchdog = None
            if holder["rig"] is not None:
                STATE["views"] = holder["rig"].close()
                holder["rig"] = None

    def tick_scenario(self, timestamp):
        """ScenarioRunner 0.9.16's ScenarioManager._tick_scenario, plus the pose batch before the
        world tick and the capture after it. Exactly `horizon` frames are filmed: a scenario tree
        that finishes earlier is not ticked again (ticking a finished tree restarts it), and one
        still running at the horizon is stopped there. A capture failure ends the run cleanly."""
        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds
            self._watchdog.update()
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()
            if self._agent is not None:
                ego_action = self._agent()  # pylint: disable=not-callable
                self.ego_vehicles[0].apply_control(ego_action)
            if holder["tree"] is None:
                self.scenario_tree.tick_once()
                status = self.scenario_tree.status
                if status != py_trees.common.Status.RUNNING:
                    holder["tree"] = str(status)
                    if status == py_trees.common.Status.FAILURE:
                        self._running = False
        if self._sync_mode and self._running and self._watchdog.get_status():
            if holder["k"] >= horizon:
                self._running = False
                return
            world = CarlaDataProvider.get_world()
            try:
                flush_poses(GameTime.get_time())
                capture(world, world.tick())
            except Exception as e:                   # noqa: BLE001
                STATE["capture_error"] = f"{type(e).__name__}: {e}"
                self._running = False
            if holder["k"] >= horizon:
                self._running = False

    ScenarioManager.run_scenario = run_scenario
    ScenarioManager._tick_scenario = tick_scenario
    _orig_sr_run = sr_main.ScenarioRunner.run

    def sr_run(self):
        result = _orig_sr_run(self)
        STATE["sr_returned"] = True
        return result
    sr_main.ScenarioRunner.run = sr_run

    sys.argv = ["scenario_runner.py", "--openscenario", str(xosc), "--sync", "--frameRate",
                str(int(round(1.0 / DT))), "--timeout", str(args.timeout), "--host", args.host,
                "--port", str(args.port), "--trafficManagerPort", str(args.tm_port)]
    t0 = time.time()
    log_path = out_dir / "scenario_runner.log"
    with open(log_path, "w") as log:
        so, se = sys.stdout, sys.stderr
        sys.stdout = sys.stderr = Tee(so, log)
        try:
            rc = sr_main.main()
        finally:
            sys.stdout, sys.stderr = so, se
    wall = time.time() - t0
    text = log_path.read_text(errors="replace")
    views_ok = rig_spec is None or (STATE["views"] and all(
        v["frames"] == horizon and v["missed"] == 0 and v["ffmpeg_rc"] == 0
        for v in STATE["views"].values()))
    completed = (STATE["sr_returned"] and STATE["capture_error"] is None
                 and STATE["frames"] >= horizon > 0)
    ok = completed and not STATE["spawn"]["lost"] and bool(views_ok)
    report = {
        "xosc": str(xosc), "clip": clip, "map": X["map"], "ok": ok,
        # ScenarioRunner prints "Not all scenario tests were successful" for any scenario
        # without test criteria (these have none): completion is the frame count instead
        "completed": completed, "sr_returned": STATE["sr_returned"], "sr_main_rc": rc,
        "capture_error": STATE["capture_error"],
        # the scenario tree's own end (all stories done) or None when stopped at the horizon
        "scenario_tree": holder["tree"],
        "wall_s": round(wall, 1), "frames_captured": STATE["frames"], "frames_expected": horizon,
        "entities_in_file": len(X["entities"]), "actors_in_run": len(holder["ids"]),
        "spawn": {"stock_refused": len(STATE["spawn"]["stock_refused"]),
                  "respawned": STATE["spawn"]["respawned"],
                  "beyond_physics_bound": STATE["spawn"]["beyond_bound"],
                  "lost": STATE["spawn"]["lost"]},
        "missing_entities": sorted(set(X["entities"]) - set(holder["ids"]))[:20],
        "colors_as_file": (sum(STATE["colors"].values()), len(STATE["colors"])),
        "weather": STATE["weather"], "lamps": STATE["lamps"],
        "pose_batch_errors": STATE["batch_errors"],
        "planner_segments_capped": STATE["planner_capped"],
        "pose_audit": audit.report(), "views": STATE["views"],
        "sr_warnings": [ln for ln in text.splitlines() if "WARNING" in ln][:30],
    }
    (out_dir / "run.json").write_text(json.dumps(report, indent=1))
    print(json.dumps({k: report[k] for k in ("clip", "ok", "completed", "wall_s",
                                             "frames_captured", "entities_in_file",
                                             "actors_in_run", "spawn", "colors_as_file",
                                             "pose_audit")}, indent=1))
    return 0 if ok else 1


class Tee:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, s):
        for st in self.streams:
            st.write(s)
        return len(s)

    def flush(self):
        for st in self.streams:
            st.flush()


def lamps_dark(world, prefix: str, heads_json: Path) -> dict:
    """Munich server v5 (-GermanLightControl=<prefix>): every German lamp head dark before t = 0,
    as in the munich2drive clips. One request file, answered by the server within a few ticks."""
    import uuid
    heads = json.loads(heads_json.read_text())["heads"]
    rid = uuid.uuid4().hex
    req = {"id": rid, "command": "lamps", "changes": [
        {"path": pair[k], "lit": False} for h in heads.values()
        for pair in h["lamps"].values() for k in ("on", "off")]}
    tmp = Path(prefix + ".request.tmp")
    tmp.write_text(json.dumps(req))
    os.replace(tmp, prefix + ".request.json")
    for i in range(20):
        time.sleep(0.06)
        world.tick()
        try:
            r = json.loads(Path(prefix + ".response.json").read_text())
        except (FileNotFoundError, ValueError):
            continue
        if r.get("id") == rid:
            return {"heads": len(heads), "ok": bool(r.get("ok")), "ticks": i + 1}
    return {"heads": len(heads), "ok": False, "ticks": 20}


if __name__ == "__main__":
    sys.exit(main())
