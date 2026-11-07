# Running and recording

```bash
python scripts/run_osc.py <file>.xosc --port 4100 --out runs/ \
    --sr-root ~/scenario_runner --carla-pythonapi ~/carla_servers/<server>/PythonAPI/carla
# Munich server only, to switch its German lamp heads dark as in the clips:
#   --lamp-bridge <server>/lights_bridge/4100/control --lamp-heads <server>/german_light_heads.json
```

`run_osc.py` runs ScenarioRunner 0.9.16 synchronously at 10 Hz (`--sync --frameRate 10`), so
there is one tick per scenario step. ScenarioRunner itself is not modified; the script patches it
in-process:

| Patch | Why |
|---|---|
| weather | The scenario's exact CARLA weather (`Carla_*` parameters) is set before the actors spawn, and again where ScenarioRunner applies the `Environment`. ScenarioRunner itself rebuilds the weather from CARLA defaults and sets only sun, cloudiness, fog distance and rain; a cloudy scenario then gets a black sky on these maps. |
| time | OpenSCENARIO's `SimulationTimeCondition` is absolute simulation time. ScenarioRunner starts its clock when the event starts, which makes every `DeleteEntityAction` fire 0.2 s late. Here the conditions count from t = 0; `TimeOut`, which ScenarioRunner uses for condition delays and its own timeout, keeps its clock. |
| spawning | ScenarioRunner spawns every agent 0.2 m above its pose and only warns when CARLA refuses one, which happens when its body touches the road or another car (4–11 agents of 120–190 per scenario). Refused agents are spawned again with more lift, then high in the air. Agents beyond UE4's 10.48 km physics bound (Munich's east end) are spawned inside it and moved out; otherwise CARLA reports a 0 × 0 × 0 bounding box for them. |
| route planner | ScenarioRunner builds CARLA's `GlobalRoutePlanner` for every map. On Belmont, three road segments never reach their end by `next()`, and the planner's loop grows until the process is killed. Here every segment's walk is capped. |
| ticks | Every actor's pose comes from the replay controller's placement (`osc/osc_replay_control.py`): all poses for a tick go out in one synchronous batch right before the world tick. A controller's own `set_transform` is asynchronous and once in ~37,000 actor-frames landed after the tick. Before the first tick every actor is placed, the Munich lamp heads go dark and the cameras are attached; one extra tick films t = 0. After each tick the script waits for every camera's image of exactly that frame, so no frame is lost or doubled. |

**Placement.** It is the same as in the munich2drive clips, with physics off:

- **ego:** its bounding-box centre goes on the trajectory point (EDGAR's origin is its rear axle);
- **other vehicles:** their origin goes on the point;
- **height, pitch and roll:** from the road under the origin, with the clips' stacked-road resolver
  choosing the deck.

## Output

```
runs/<clip>/
├── front.mp4  front_left.mp4  front_right.mp4  back.mp4  back_left.mp4  back_right.mp4
├── bird.mp4   chase.mp4
├── <view>_edge.mp4      one edge-detection video per view
├── run.json             completion, spawn, colours, weather, pose audit, frames per view
└── scenario_runner.log
```

All videos are 10 fps and 200 frames (20 s): frame *k* shows simulation time *k* × 0.1 s.

**Cameras** (`rigs/edgar_rig.json`): the camera rig of the munich2drive clips, on the EDGAR ego.

- **Surround views:** the six are EDGAR's real cameras, at their poses from `edgar.urdf`, 960×600.
- **Intrinsics:** each of the six is rendered slightly larger, with its own field of view and roll,
  then cropped. That reproduces the real focal length and principal-point offset. The front cameras
  are ~85°, the rear ones ~100°.
- **`bird` and `chase`:** 1280×720 visualisation views, 25 m above the ego and behind it.
- **Gamma:** 2.2.

**Edge view:** Canny edge detection on every frame **before** compression, so the edges are the
scene's, not the codec's: grayscale → ffmpeg `edgedetect` (low 0.08, high 0.25), white edges on black.
`--edge-low` and `--edge-high` change the thresholds.

**`run.json`:**

- `ok`: true only when the run completed with every agent and every frame of every view.
- `completed`, `frames_captured` / `frames_expected`, and `capture_error` if a capture failed.
- `spawn`: agents CARLA refused and how they were recovered; `lost` must be empty.
- `colors_as_file`: actors whose colour is the one in the file, out of those with a colour.
- `weather`, `lamps`, and `planner_segments_capped`.
- `pose_audit`:
  - `max_m`, `p99_m`, `median_m`: each actor's largest distance between its reference point (the
    ego's box centre, every other vehicle's origin) and its trajectory, summarised over all actors;
  - `max_dz_m`: the largest height error against the placement;
  - `worst`: the five worst actors;
  - `frames_over_5cm`: up to 30 frames with any actor more than 5 cm off;
  - `in_view_after_delete`: frames in which an actor was still visible after its removal.
- `views`: frames, missed frames and the ffmpeg exit code per view.

The exit status is 0 only when `ok` is true.

## Plain ScenarioRunner

```bash
python scenario_runner.py --openscenario <file>.xosc --sync --frameRate 10 --timeout 300
```

This runs a file only if CARLA spawns every agent. CARLA refuses an agent whose body touches the road
or another car at spawn, which happened in 5 of the 6 test variants (4–11 agents each). ScenarioRunner
then aborts the scenario:

- at load, with "Cannot find actor", when the file later removes the refused agent;
- otherwise when the refused agent's trajectory starts.

On Belmont, ScenarioRunner also never finishes building its route planner; the process grows until it
is killed. Use `run_osc.py` instead, with `--no-record` to run without cameras: it spawns refused
agents again and caps the planner.

When plain ScenarioRunner does run a file, the controller places every agent, reads its trajectory
from the file, and takes a removed agent out of view on time. Only the sun position, cloudiness and
rain amount of the weather are reproduced. `scripts/sr_live_check.py` runs plain ScenarioRunner on a
file and audits the poses from a second client.

## Your own ego stack

Set the `hero` controller's `module` property to `external_control` (Storyboard > Init > Private
`hero`) and delete the `ManeuverGroup` named `mg_hero`. `run_osc.py` places only agents that have a
trajectory in the file, so the ego is left to your controller and the cameras still follow it. The ego
spawns with its CARLA origin on the Init position, which is the footprint centre. EDGAR's origin is
its rear axle, so move the `hero` Init `WorldPosition` 1.52 m back along its heading:
x − 1.52·cos h, y − 1.52·sin h. The other agents keep their recorded motion and do not react to the ego.

## Comparing with the munich2drive clip

```bash
python scripts/compare_clip.py runs/scenario_00_ego_5105 <folder with the clip's videos>
```

This gives PSNR and SSIM per view, frame by frame (`compare.json`). Expect 25–33 dB and 0.88–0.98
even when every pose agrees to the millimetre: CARLA animates vegetation in the wind in real time,
lens flare and temporal anti-aliasing differ between two runs, and the traffic lights run CARLA's
own cycles in a re-run.

## Speed

On an RTX 5090 with two servers on the card, one variant with all 16 videos takes 4–13 minutes,
depending on the number of agents and the map. Without cameras (`--no-record`) a run takes under a
minute after the map has loaded.
