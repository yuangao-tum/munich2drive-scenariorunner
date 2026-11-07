"""ScenarioRunner actor controller: replays a timed OpenSCENARIO trajectory exactly (kinematic).

Shipped with the munich2drive OpenSCENARIO scenarios, where every entity is assigned it in Init
(<Property name="module" value="osc_replay_control"/>, plus
<Property name="trajectory_file" value="<this .xosc>"/>). ScenarioRunner puts the .xosc's
directory on sys.path, so this file sits next to the .xosc.

Trajectory: read from the scenario file itself -- the file ScenarioRunner was started with
(--openscenario), else the one named by `trajectory_file` -- so the actor drives from its FIRST
tick. Waiting for the FollowTrajectoryAction hand-over costs two ticks at the start, in which
every agent would stand still (~5 m behind at 0.2 s on Munich). Without a readable file the
controller falls back to the hand-over's waypoints and times (ScenarioRunner 0.9.16; earlier
versions hand over no times, and the controller then refuses instead of guessing).

Placement, with physics off, as in the munich2drive clips:
  * x, y: the ego ("hero") with its bounding-box CENTRE on the footprint point (EDGAR's CARLA
    origin is its rear axle, 1.52 m behind the centre); every other vehicle with its ORIGIN on
    the point (CARLA models' origins lie within a few cm of their box centres, except the
    ambulance, 0.29 m, and the tourismo bus, 0.18 m).
  * z, pitch, roll: the road under the actor's origin, picked per trajectory with the clips'
    stacked-road resolver (probes around the map's reference height, lane position and heading
    agreement, continuity along the trajectory; where nothing decides, the file's z). The
    origin is lifted by the model's bounding-box bottom offset so the wheels sit on the road.
  * time: GameTime during a tick is already the time of the frame that tick produces
    (ScenarioManager adds the snapshot's delta first, then ticks the tree, then the world).
  * removal: an agent the file removes (DeleteEntityAction) is taken out of view at that time.
    ScenarioRunner starts a time condition's clock when its event starts, so its own delete
    comes 0.2 s late; the actor is then already hidden.
"""
import bisect
import math
import os
import sys
import xml.etree.ElementTree as ET

import carla

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime

# Set by a host script that places every actor itself, in one synchronous batch per tick (the
# munich2drive-scenariorunner run_osc.py does): the controller then does nothing. A controller's
# set_transform is asynchronous; measured once in ~37,000 actor-frames it landed after the tick.
EXTERNAL_PLACEMENT = False

# the clips' stacked-road resolver
Z_PROBE_OFFSETS = (-20.0, -12.0, -6.0, 0.0, 6.0, 12.0, 20.0)
Z_LAYER_TOL = 0.5           # decks closer than this are the same carriageway (m)
Z_LANE_XY_TOL = 0.6         # a lane centre this close to the point carries the vehicle (m)
Z_HEADING_TOL_DEG = 45.0    # lane heading vs vehicle heading

# EDGAR on the munich2drive servers (vehicle.lincoln.mkz_2017): bounding-box centre 1.5203 m
# ahead of the origin, box bottom at the origin. Used when CARLA reports a 0 x 0 x 0 box, which
# it does for an actor spawned beyond UE4's 10.48 km physics bound (Munich's east end).
EDGAR_BP = "vehicle.lincoln.mkz_2017"
EDGAR_BOX = (1.5203, 0.0, 0.0)
HIDE_Z = -1000.0            # below the map: out of every camera's view

_FILES: dict = {}
_REF_Z: dict = {}


# ------------------------------------------------------------------ the file
def find_file(name):
    """`name` as given, else in ScenarioRunner's scenario folder, else on sys.path (which
    ScenarioRunner extends with the scenario's folder before importing this module)."""
    if not name:
        return None
    if os.path.isabs(name) and os.path.exists(name):
        return name
    dirs = []
    try:
        from srunner.tools.openscenario_parser import OpenScenarioParser
        dirs.append(getattr(OpenScenarioParser, "osc_filepath", None) or "")
    except ImportError:
        pass
    for d in dirs + list(sys.path):
        cand = os.path.join(d or ".", name)
        if os.path.exists(cand):
            return cand
    return None


def scenario_file(named=None):
    """The scenario being run: ScenarioRunner's --openscenario argument (an edited copy is read,
    not the original it was made from), else the file `named` (trajectory_file)."""
    argv = sys.argv
    if "--openscenario" in argv:
        i = argv.index("--openscenario")
        if i + 1 < len(argv) and os.path.exists(argv[i + 1]):
            return argv[i + 1]
    return find_file(named)


def read_file(path):
    """{entity: {"t": [s], "pts": [(x, y, z, yaw_deg)] CARLA frame, "gone": s or None}} for every
    FollowTrajectoryAction (y and heading negated, as ScenarioRunner converts a WorldPosition)
    and the entity's DeleteEntityAction time."""
    if path not in _FILES:
        out = {}
        for mg in ET.parse(path).getroot().iter("ManeuverGroup"):
            ref = mg.find("Actors/EntityRef")
            if ref is None:
                continue
            ts, pts, gone = [], [], None
            for v in mg.iter("Vertex"):
                wp = v.find("Position/WorldPosition")
                if wp is None or v.get("time") is None:
                    continue
                ts.append(float(v.get("time")))
                pts.append((float(wp.get("x")), -float(wp.get("y")), float(wp.get("z", 0.0)),
                            -math.degrees(float(wp.get("h", 0.0)))))
            for ev in mg.iter("Event"):
                if ev.find(".//DeleteEntityAction") is not None:
                    c = ev.find(".//SimulationTimeCondition")
                    if c is not None:
                        gone = float(c.get("value"))
            if ts:
                out[ref.get("entityRef")] = {"t": ts, "pts": pts, "gone": gone}
        _FILES[path] = out
    return _FILES[path]


# ------------------------------------------------------------------ placement
def replay_box(actor, role=None):
    """(ox, oy, z_off): the origin offset from the footprint point (actor frame) and the origin
    height above the road. hero: box centre on the point; every other vehicle: origin on it."""
    role = role or actor.attributes.get("role_name")
    bb = actor.bounding_box
    if bb.extent.x < 0.05:                      # 0 x 0 x 0: spawned beyond the physics bound
        return EDGAR_BOX if (role == "hero" and actor.type_id == EDGAR_BP) else (0.0, 0.0, 0.0)
    z_off = float(bb.extent.z - bb.location.z)
    if role == "hero":
        return (float(bb.location.x), float(bb.location.y), z_off)
    return (0.0, 0.0, z_off)


def map_ref_z(cmap):
    """The resolver's reference height: the median spawn-point height of the map (the median
    waypoint height on a map without spawn points)."""
    if cmap.name not in _REF_Z:
        sps = cmap.get_spawn_points()
        if sps:
            z = sorted(s.location.z for s in sps)[len(sps) // 2]
        else:
            wz = sorted(w.transform.location.z for w in cmap.generate_waypoints(20.0))
            z = wz[len(wz) // 2] if wz else 0.0
        _REF_Z[cmap.name] = float(z)
    return _REF_Z[cmap.name]


def _resolver(cmap, ref_z):
    """resolve(origin points, yaws, file z) -> [(z, pitch, roll)] per state."""
    def get_wp(cx, cy, hint):
        wp = cmap.get_waypoint(carla.Location(x=cx, y=cy, z=hint), project_to_road=True)
        if wp is None:
            return None
        tr = wp.transform
        return (float(tr.location.z), float(tr.rotation.pitch), float(tr.rotation.roll),
                float(tr.location.x), float(tr.location.y), float(tr.rotation.yaw))

    def cands(cx, cy):
        out = []
        for off in Z_PROBE_OFFSETS:
            g = get_wp(cx, cy, ref_z + off)
            if g is not None and all(abs(g[0] - c[0]) > Z_LAYER_TOL for c in out):
                out.append(g)
        return out

    def resolve(pts, yaws, file_z):
        n = len(pts)
        C = [cands(cx, cy) for cx, cy in pts]
        pick = [None] * n
        for i, ((cx, cy), c, yaw) in enumerate(zip(pts, C, yaws)):
            if not c:                                        # off the road network
                pick[i] = (file_z[i], 0.0, 0.0, cx, cy, None)
                C[i] = [pick[i]]
            elif len(c) == 1:
                pick[i] = c[0]
            else:
                on = [g for g in c if math.hypot(g[3] - cx, g[4] - cy) < Z_LANE_XY_TOL]
                if len(on) == 1:
                    pick[i] = on[0]
                elif len(on) > 1:
                    h = [g for g in on if g[5] is not None and
                         abs((g[5] - yaw + 180.0) % 360.0 - 180.0) < Z_HEADING_TOL_DEG]
                    if len(h) == 1:
                        pick[i] = h[0]
        anchor = next((i for i in range(n) if pick[i] is not None), None)
        if anchor is None:       # nothing decides anywhere: the deck the file's z is on
            return [(g[0], g[1], g[2]) for g in
                    (min(c, key=lambda g, fz=fz: abs(g[0] - fz)) for c, fz in zip(C, file_z))]
        for rng in (range(anchor + 1, n), range(anchor - 1, -1, -1)):
            for i in rng:
                if pick[i] is None:
                    ref = pick[i - 1][0] if i > anchor else pick[i + 1][0]
                    pick[i] = min(C[i], key=lambda g, r=ref: abs(g[0] - r))
        return [(g[0], g[1], g[2]) for g in pick]

    return resolve


def resolve_track(cmap, traj, box, ref_z=None):
    """Per vertex (x, y, z, yaw, pitch, roll) of the actor ORIGIN in the CARLA frame."""
    ox, oy, z_off = box
    origin = []
    for x, y, _z, yaw in traj["pts"]:
        psi = math.radians(yaw)
        origin.append((x - (ox * math.cos(psi) - oy * math.sin(psi)),
                       y - (ox * math.sin(psi) + oy * math.cos(psi))))
    ground = _resolver(cmap, map_ref_z(cmap) if ref_z is None else ref_z)(
        origin, [p[3] for p in traj["pts"]], [p[2] for p in traj["pts"]])
    return [(x, y, g[0] + z_off, p[3], g[1], g[2])
            for (x, y), g, p in zip(origin, ground, traj["pts"])]


def pose_at(ts, track, t):
    """carla.Transform of the origin at time t: linear between vertices, angles the short way,
    held at the ends."""
    if len(ts) == 1 or t <= ts[0]:
        a = b = track[0]
        w = 0.0
    elif t >= ts[-1]:
        a = b = track[-1]
        w = 0.0
    else:
        i = bisect.bisect_right(ts, t) - 1
        w = (t - ts[i]) / (ts[i + 1] - ts[i])
        a, b = track[i], track[i + 1]

    def lin(u, v):
        return u + (v - u) * w

    def ang(u, v):
        return u + ((v - u + 180.0) % 360.0 - 180.0) * w
    return carla.Transform(carla.Location(x=lin(a[0], b[0]), y=lin(a[1], b[1]), z=lin(a[2], b[2])),
                           carla.Rotation(pitch=ang(a[4], b[4]), yaw=ang(a[3], b[3]),
                                          roll=ang(a[5], b[5])))


# ------------------------------------------------------------------ the controller
class OscReplayControl(BasicControl):
    """Places its actor on the timed trajectory of the scenario file at every tick."""

    def __init__(self, actor, args=None):
        super().__init__(actor)
        self._args = args or {}
        self._role = actor.attributes.get("role_name")
        named = self._args.get("trajectory_file")
        path = scenario_file(named)
        self._traj = read_file(path).get(self._role) if path else None
        if named and path is None:
            print(f"WARNING: osc_replay_control: {named} not found -- following the "
                  f"FollowTrajectoryAction hand-over, which starts 2 ticks late", flush=True)
        self._ts = self._track = None
        self._hidden = self._dead = False

    def reset(self):
        """Nothing to undo: the actor is left where the replay put it."""

    def update_waypoints(self, waypoints, times=None, start_time=None):
        """FollowTrajectoryAction's hand-over. Ignored when the file was read and holds the same
        polyline; followed when there is no file or the scenario's polyline differs."""
        if self._traj is not None and len(waypoints) == len(self._traj["t"]):
            self._start_time = start_time
            return
        if self._traj is not None:
            print(f"WARNING: osc_replay_control: {self._role}: the scenario hands over "
                  f"{len(waypoints)} vertices, the file holds {len(self._traj['t'])} -- "
                  f"following the scenario", flush=True)
        if not times or len(times) != len(waypoints):
            raise RuntimeError("osc_replay_control: this ScenarioRunner passes no vertex times "
                               "with FollowTrajectoryAction (it needs ScenarioRunner 0.9.16)")
        super().update_waypoints(waypoints, times, start_time)
        self._reached_goal = False
        gone = self._traj.get("gone") if self._traj else None
        self._traj = {"t": list(times), "gone": gone,
                      "pts": [(w.location.x, w.location.y, w.location.z, w.rotation.yaw)
                              for w in waypoints]}
        self._track = None

    def run_step(self):
        if self._traj is None:
            return
        t = GameTime.get_time()
        # ScenarioRunner ends a scenario when every story is done, and a FollowTrajectoryAction
        # is done when its controller reports the goal reached: report it in every mode (it
        # ignores a storyboard StopTrigger that is not a criterion)
        if t >= self._traj["t"][-1]:
            self._reached_goal = True
        if self._dead or EXTERNAL_PLACEMENT:
            return
        try:
            gone = self._traj.get("gone")
            if gone is not None and t > gone:
                if not self._hidden:
                    tf = self._actor.get_transform()
                    tf.location.z = HIDE_Z
                    self._actor.set_transform(tf)
                    self._hidden = True
                return
            if self._track is None:
                cmap = CarlaDataProvider.get_map()
                self._actor.set_simulate_physics(False)
                self._ts = self._traj["t"]
                self._track = resolve_track(cmap, self._traj, replay_box(self._actor, self._role))
            self._actor.set_transform(pose_at(self._ts, self._track, t))
        except RuntimeError as err:
            if "destroyed actor" in str(err):       # ScenarioRunner's delete got there first
                self._dead = True
                return
            raise
