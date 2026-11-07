# munich2drive ScenarioRunner

Run the **munich2drive OpenSCENARIO scenarios** in CARLA 0.9.16 with **ScenarioRunner 0.9.16**,
and record the ego's camera views together with an **edge-detection** video of every view.

The scenarios themselves (Munich and Bay Area maps, base scenarios and ego variants) and how they are
generated are in **munich2drive-openscenario**. This repository is only about running and rendering
them.

| Map family | Maps | CARLA 0.9.16 server (private Hugging Face dataset repo) |
|---|---|---|
| Munich | `Munich_Route_1` | `Yuan-avs/Munich_Route_1_carla0916` |
| Bay Area | `Page_Mill_Rd_Palo_Alto_CA`, `Yale_St_Palo_Alto_CA`, `Belmont_Office_Park_Belmont_CA` | `TUM-AVS/BayArea_carla0916` |

## Quick start

```bash
# 0. environment: Python 3.10 or 3.11 (details: docs/setup.md)
sudo apt install ffmpeg git libvulkan1
python3.10 -m venv ~/m2d-sr && . ~/m2d-sr/bin/activate
pip install -U pip "huggingface_hub>=0.25" && hf auth login

# 1. a CARLA server with the maps (private repo: ask for access)
python scripts/download_server.py bayarea --dest ~/carla_servers
scripts/start_carla.sh ~/carla_servers/BayArea_carla0916_server_v1 4100        # keep running

# 2. ScenarioRunner 0.9.16 and its Python packages
git clone --branch v0.9.16 https://github.com/carla-simulator/scenario_runner.git ~/scenario_runner
pip install carla==0.9.16 -r ~/scenario_runner/requirements.txt

# 3. scenario files (private dataset), then one run: 8 camera views + 8 edge views, 200 frames each
hf download <scenario dataset> --repo-type dataset \
    --include "Page_Mill_Rd_Palo_Alto_CA/*" --local-dir ~/m2d_osc
python scripts/run_osc.py ~/m2d_osc/Page_Mill_Rd_Palo_Alto_CA/variants/scenario_00_ego_5105.xosc \
    --port 4100 --out runs/ --sr-root ~/scenario_runner \
    --carla-pythonapi ~/carla_servers/BayArea_carla0916_server_v1/PythonAPI/carla
```

The output is in `runs/scenario_00_ego_5105/`:

- `front.mp4` and `front_edge.mp4`, and the same for all 8 views;
- `run.json`: completion, spawn check, colours, weather, pose audit, frames per view.

`run_osc.py` exits with 0 only when the run completed with every agent and every frame.

To compare a run with the original munich2drive clip:

```bash
python scripts/compare_clip.py runs/<clip> <folder with the clip's videos>
```

## How it runs

The scenario files are plain OpenSCENARIO 1.0 and load in stock ScenarioRunner 0.9.16. Every agent
is driven by `osc_replay_control.py`, which ships next to the scenario files and in `osc/` here. It
places each agent where its recorded trajectory is at every tick, with physics off, the way the clips
were rendered. ScenarioRunner's default controller would drive the same paths with physics, and dense
traffic drifts off them.

`run_osc.py` runs ScenarioRunner unmodified and patches it in-process:

- **weather:** the scenario's exact CARLA weather, before the actors spawn;
- **time:** absolute time conditions, so agents are removed on time;
- **spawning:** recovers agents CARLA refuses to spawn, and those beyond the physics bound;
- **route planner:** a fix for a loop on the Belmont map;
- **ticks:** one synchronous pose batch per tick, and every camera frame waited for.

See [docs/recording.md](docs/recording.md), which also covers driving the ego with your own stack.
Use `run_osc.py` (`--no-record` without cameras) rather than plain `scenario_runner.py`: plain
ScenarioRunner aborts a scenario when CARLA refuses to spawn one of its agents, which happened in 5
of the 6 test variants.

## Documentation

- [docs/setup.md](docs/setup.md): environment, servers from Hugging Face, ScenarioRunner, scenario files
- [docs/munich.md](docs/munich.md), [docs/bayarea.md](docs/bayarea.md): running on each map family
- [docs/recording.md](docs/recording.md): cameras, edge views, outputs, plain ScenarioRunner, comparison
- [docs/test-report.md](docs/test-report.md): the test on both map families (6 variants, 1 base scenario)

## Layout

```
scripts/download_server.py   CARLA server from Hugging Face: download, SHA-256, unpack, check
scripts/start_carla.sh       start a server headless with the munich2drive flags
scripts/run_osc.py           ScenarioRunner + cameras + edge views + pose audit
scripts/compare_clip.py      PSNR/SSIM of a run against the munich2drive clip
scripts/report_runs.py       markdown table of several runs
scripts/sr_live_check.py     plain ScenarioRunner run, audited from a second client
osc/osc_replay_control.py    the ScenarioRunner controller the scenario files assign
rigs/edgar_rig.json          the EDGAR camera rig of the munich2drive clips
```
