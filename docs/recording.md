# Running and recording

```bash
python scripts/run_osc.py <file>.xosc --port 4100 --out runs/ \
    --sr-root ~/scenario_runner --carla-pythonapi ~/carla-0.9.16-src/PythonAPI/carla
# Munich server only, to switch its German lamp heads dark as in the clips:
#   --lamp-bridge <server>/lights_bridge/4100/control --lamp-heads <server>/german_light_heads.json
# ... and to let them show the scenario's signals, as in the clips (see "Lamp plans" below):
#   --lamp-plan <set>/openscenario/lamps/scenario_<id>.json
```

`run_osc.py` runs ScenarioRunner 0.9.16 synchronously at 10 Hz (`--sync --frameRate 10`), so
there is one tick per scenario step. ScenarioRunner itself is not modified; the script patches it
in-process:

| Patch | Why |
|---|---|
| weather | The scenario's exact CARLA weather (`Carla_*` parameters) is set before the traffic spawns (the ego comes first) and long before the first frame, and again where ScenarioRunner applies the `Environment`. ScenarioRunner itself rebuilds the weather from CARLA defaults and sets only sun, cloudiness, fog distance and rain; a cloudy scenario then gets a black sky on these maps. |
| time | OpenSCENARIO's `SimulationTimeCondition` is absolute simulation time. ScenarioRunner starts its clock when the event starts, which makes every `DeleteEntityAction` fire 0.2 s late. Here the conditions count from t = 0; `TimeOut`, which ScenarioRunner uses for condition delays and its own timeout, keeps its clock. |
| spawning | ScenarioRunner spawns every agent 0.2 m above its pose and only warns when CARLA refuses one, which happens when its body touches the road or another car (0–48 agents per variant in the test, of 83–189). Refused agents are spawned again with more lift, then high in the air. Agents beyond UE4's 10.48 km physics bound (Munich's east end) are spawned inside it and moved out; otherwise CARLA reports a 0 × 0 × 0 bounding box for them. |
| route planner | ScenarioRunner builds CARLA's `GlobalRoutePlanner` for every map, twice per run. On Belmont, three road segments never reach their end by `next()`, and the planner's loop grows until the process is killed. Here every segment's walk is capped; the cap also cuts three Munich segments short. A replay does not use the planner: ScenarioRunner follows a trajectory's vertices as a "shortest" route. |
| lamps | With `--lamp-plan`, the Munich lamp heads show the scenario's lamp plan after they went dark: before the tick that films frame *k*, the heads whose state changes at step *k* are switched, and the server's reply after the tick proves the change is on frame *k*. |
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
├── trajectory.json      the ego at every frame, in the clips' columns
├── lamps.json           what each lamp head showed (--lamp-plan only)
├── run.json             completion, spawn, colours, weather, pose audits, lamps, frames per view
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

- `ok`: true only when every entry of `checks` is true (`failed_checks` lists the others):
  - `completed`: all `frames_expected` frames (the file's `HorizonSteps`) were captured;
  - `views_complete`: every view has every frame, none missed, and ffmpeg succeeded;
  - `every_agent_spawned`: no agent lost at spawn;
  - `every_actor_in_every_frame`: no actor missing from a frame before its removal;
  - `removed_on_time`: no actor visible after its removal;
  - `poses_within_tolerance`: every actor within `--pose-tolerance` (5 cm) of the file, in position
    and height;
  - `ego_pose_within_tolerance`: the ego's realized pose within 0.10 m and 2° of its placement at
    every frame, in position, height, tilt and heading (`ego_pose_audit`);
  - `pose_batches_applied`: CARLA applied every pose;
  - `frames_in_step`: every frame one tick and 0.1 s after the previous one (another client ticking
    the same server breaks this);
  - `lamps_switched`: the Munich lamp heads switched (`--lamp-bridge` only);
  - `lamps_followed`: the lamp heads showed the plan on every frame (`--lamp-plan` only).
- `completed`, `frames_captured` / `frames_expected`, and `capture_error` if a capture failed;
  `frame_errors` with examples when frames were out of step.
- `spawn`: agents CARLA refused and how they were recovered; `lost` must be empty.
- `colors_as_file`: actors whose colour is the one in the file, out of those with a colour.
- `weather`, `lamps`, and `planner_segments_capped` (road segments whose walk was capped).
  With `--lamp-plan`, `lamps.plan` counts the changes made and planned, requests whose reply missed
  their tick or was refused (that change then went out with the next frame), and `followed`.
- `ego_pose_audit`: the ego's realized pose against its placement at every frame: the worst
  horizontal and vertical offset, tilt (the angle between the up axes) and heading difference,
  the frames over 0.10 m / 2°, and frames without the ego.
- `pose_audit`:
  - `max_m`, `p99_m`, `median_m`: each actor's largest distance between its reference point (the
    ego's box centre, every other vehicle's origin) and its trajectory, summarised over all actors;
  - `max_dz_m`: the largest height error against the placement;
  - `worst`: the five worst actors;
  - `frames_over_5cm`: up to 30 frames with any actor more than 5 cm off;
  - `missing_actor_frames`: actor-frames in which an actor was absent before its removal;
  - `in_view_after_delete`: actor-frames in which an actor was still visible after its removal.
- `views`: frames, missed frames and the ffmpeg exit code per view.

The exit status is 0 only when `ok` is true.

**`trajectory.json`:** the ego (`hero`) at every frame, in the columns of the munich2drive clips'
`<prefix>_trajectory.json`, under `trajectory`:

| Columns | Meaning |
|---|---|
| `t` | the frame (step) |
| `x`, `y`, `yaw` | the file's trajectory point (the ego's box centre), in the file's frame: x east, y north, yaw in rad. On these maps this is the clips' CommonRoad frame. |
| `carla_x`, `carla_y` | the same point in the CARLA frame (y flipped) |
| `z`, `pitch`, `roll`, `carla_yaw_deg` | the placed pose: road height under the actor origin (the origin sits `ego_z_off_m` above it), pitch, roll and yaw |
| `rear_axle_x`, `rear_axle_y` (+ `_carla_`) | where the actor origin was placed; EDGAR's origin is its rear axle |
| `realized_x` … `realized_roll_deg` | the actor transform CARLA reported after the tick: CARLA frame, actor origin |

The file has no speeds, so the clips' `v`, `a` and `yaw_rate` are not written. The header gives the
ego's blueprint, bounding box, `ego_origin_shift_m` (box centre relative to the origin) and whether
the file replays it (`controlled`: `replayed`) or your controller drives it (`external`: only the
`realized_*` columns). A base scenario without an ego writes no `trajectory.json`.

## Lamp plans (Munich)

In the munich2drive clips the German lamp heads follow the scenario's traffic signals. A `.xosc`
file has no signal states, so the Munich sets come with one lamp plan per scenario, in
`openscenario/lamps/scenario_<id>.json` next to `base/` and `variants/`. One plan serves the base
scenario and all its variants: variant time 0 is scenario step 0.

```bash
python scripts/run_osc.py <set>/openscenario/variants/scenario_00_ego_6938.xosc ... \
    --lamp-bridge $SRV/lights_bridge/4100/control --lamp-heads $SRV/german_light_heads.json \
    --lamp-plan <set>/openscenario/lamps/scenario_00.json
```

A plan is the clips' `<prefix>_lamps.json` format:

```json
{"steps": 200, "heads": {"<head path>": [[0, "red"], [5, "green"], [96, "yellow"]], ...}}
```

- **Heads:** the keys are the head paths of `german_light_heads.json`; a run with an unknown head
  stops before it starts.
- **States:** every head starts dark and changes at step `t` to the state given: `green`, `yellow`,
  `red`, `red_yellow` or `off`. A head that never lights is not listed and stays dark, like the
  heads no signal reaches in the clips.
- **Timing:** a change at step *k* is on frame *k*. `lamps.json` in the run folder records what
  each head showed, in the same format; for a run with `lamps_followed` it equals the plan.

Needs server v5 (the only build with the lamp control) and `--lamp-bridge`.

## Plain ScenarioRunner

```bash
cd ~/scenario_runner
PYTHONPATH=~/carla-0.9.16-src/PythonAPI/carla python scenario_runner.py \
    --openscenario <file>.xosc --sync --frameRate 10 --timeout 300 --port 4100 \
    --trafficManagerPort 8100
```

This runs a file only if CARLA spawns every agent. CARLA refuses an agent whose body touches the road
or another car at spawn, which happened in 5 of the 6 test variants (4–48 agents each). ScenarioRunner
then aborts the scenario:

- at load, with "Cannot find actor", when the file later removes the refused agent;
- otherwise when the refused agent's trajectory starts.

On Belmont, ScenarioRunner also never finishes building its route planner; the process grows until it
is killed. Use `run_osc.py` instead, with `--no-record` to run without cameras: it spawns refused
agents again and caps the planner.

When plain ScenarioRunner does run a file, the controller places every agent, reads its trajectory
from the file, and takes a removed agent out of view on time. Only the sun position, cloudiness and
rain amount of the weather are reproduced. `scripts/sr_live_check.py` runs plain ScenarioRunner on a
file and audits the poses from a second client. ScenarioRunner's Traffic Manager port defaults to
8000, so give two runs on one machine different ones (`sr_live_check.py` uses the server port +
4000, as `run_osc.py`).

## Your own ego stack

Set the `hero` controller's `module` property to `external_control` (Storyboard > Init > Private
`hero`) and delete the `ManeuverGroup` named `mg_hero`. `run_osc.py` places only entities that have
a `ManeuverGroup` in the file, so the ego is left to your controller and the cameras still follow it. The ego
spawns with its CARLA origin on the Init position, which is the footprint centre. EDGAR's origin is
its rear axle, so move the `hero` Init `WorldPosition` 1.52 m back along its heading:
x − 1.52·cos h, y − 1.52·sin h. The other agents keep their recorded motion and do not react to the ego.

## Comparing with the munich2drive clip

```bash
python scripts/compare_clip.py runs/scenario_00_ego_5105 <folder with the clip's videos>
```

This gives PSNR and SSIM per view, frame by frame (`compare.json`). The test runs reached 24–41 dB
mean PSNR (single views down to 20 dB in heavy rain) and SSIM 0.90–0.98, with every pose within
1.4 mm. The differences come from rendering: rain streaks are drawn randomly, CARLA animates
vegetation in the wind in real time,
lens flare and temporal anti-aliasing differ between two runs, and the traffic lights run CARLA's
own cycles in a re-run.

## Speed

On an RTX 5090 with two servers on the card, one variant with all 16 videos takes 4–13 minutes,
depending on the number of agents and the map. Without cameras (`--no-record`) a run takes under a
minute after the map has loaded.
