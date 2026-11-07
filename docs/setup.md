# Setup

Linux with an NVIDIA GPU. One CARLA server needs about 8–9 GB of VRAM with the eight cameras
attached; two servers fit on a 24 GB card.

## 0. Environment

```bash
sudo apt install ffmpeg git libvulkan1           # ffmpeg with libx264; CARLA renders through Vulkan
python3.10 -m venv ~/m2d-sr && . ~/m2d-sr/bin/activate
pip install -U pip "huggingface_hub>=0.25"
hf auth login                                    # or: export HF_TOKEN=...
```

**Python 3.10 or 3.11.** ScenarioRunner 0.9.16 pins numpy 1.24.4, which has no wheel for 3.12.

- **Ubuntu 22.04:** `sudo apt install python3.10-venv`.
- **Ubuntu 24.04 and later:** these ship only 3.12; get 3.10 from the deadsnakes PPA
  (`sudo add-apt-repository ppa:deadsnakes/ppa && sudo apt install python3.10 python3.10-venv`),
  from conda, or with `uv venv --python 3.10`.

**Driver.** The NVIDIA driver must support Vulkan; `vulkaninfo` (package `vulkan-tools`) checks it.

**Separate environment.** Use one for this repository only: ScenarioRunner pins
`antlr4-python3-runtime==4.10`, and a package built against ANTLR 4.9 (for example `omegaconf`)
cannot be imported next to it.

## 1. A CARLA server with the maps

The maps are not part of stock CARLA. Each map family ships as a complete, packaged CARLA 0.9.16
server in a **private** Hugging Face dataset repo; ask the owner for access.

| Map family | Hugging Face repo | Archive | Unpacked |
|---|---|---|---|
| Munich Route_1 | `Yuan-avs/Munich_Route_1_carla0916` | `Munich_Route_1_carla0916_server_v5.tar.gz` (15.8 GB) | ~39 GB |
| Bay Area (Page Mill, Yale, Belmont) | `TUM-AVS/BayArea_carla0916` | `BayArea_carla0916_server_v1.tar.gz` (20.0 GB) | ~51 GB |

```bash
python scripts/download_server.py munich  --dest ~/carla_servers
python scripts/download_server.py bayarea --dest ~/carla_servers
```

The script needs the archive and the unpacked tree on disk at the same time: about 57 GB free for
Munich and 74 GB for the Bay Area. It does the following:

- downloads the archive and checks it against the SHA-256 file stored next to it in the repo;
- unpacks it to `~/carla_servers/<server folder>/` and checks that the server binary and the maps'
  OpenDRIVE files are there;
- refuses to unpack over an existing folder, and removes a partial folder if unpacking fails;
- deletes the archive afterwards (`--keep-archive` keeps it);
- for Munich, also fetches `german_light_heads.json` into the server folder.

`--archive PATH` uses an archive you already have, with the same SHA-256 check. For the Bay Area the
script tries `TUM-AVS/BayArea_carla0916` first and falls back to `Yuan-avs/BayArea_carla0916`, the
original copy.

Use each server only for its own maps. The Bay Area server is a separate build: 81 asset paths it
contains collide with Munich content, so never load Munich on it, or the reverse.

Start a server and keep it running in tmux or screen:

```bash
scripts/start_carla.sh ~/carla_servers/Munich_Route_1_carla0916_server_v5 4100
scripts/start_carla.sh ~/carla_servers/BayArea_carla0916_server_v1        4100
```

It uses the flags of the munich2drive clips: `-RenderOffScreen -quality-level=Epic -nosound`, RPC
port 4100. The server also uses 4101 and 4102. For a second server on the same machine, start it on
another port (e.g. 4110), and run `run_osc.py --port 4110` against it. The Traffic Manager port
defaults to the server port + 4000, so the two runs do not collide. On a machine with several GPUs set `CARLA_GPU=<index>`. On the Munich server the
script also switches on the control of the map's German traffic-light lamp heads, and prints the two
`run_osc.py` arguments that use it.

## 2. ScenarioRunner 0.9.16

```bash
git clone --branch v0.9.16 https://github.com/carla-simulator/scenario_runner.git ~/scenario_runner
export SCENARIO_RUNNER_ROOT=~/scenario_runner
```

Use the 0.9.16 release itself: earlier versions do not hand trajectory times to a controller, and
forks may change the OpenSCENARIO parser in other ways.

## 3. Python packages

In the environment from step 0:

```bash
pip install carla==0.9.16                        # the client must match the server
pip install -r ~/scenario_runner/requirements.txt
```

ScenarioRunner also imports CARLA's `agents` package, which is not on PyPI. Both server folders
contain it: pass `--carla-pythonapi ~/carla_servers/<server folder>/PythonAPI/carla` to
`run_osc.py`, or set `CARLA_ROOT=~/carla_servers/<server folder>`.

## 4. The scenario files

The OpenSCENARIO sets are in the private Hugging Face dataset repo
`<scenario dataset>`, one folder per map; see munich2drive-openscenario. For example:

```bash
hf download <scenario dataset> --repo-type dataset \
    --include "Yale_St_Palo_Alto_CA/*" --local-dir ~/m2d_osc
```

Keep each set's folder as it is: `osc_replay_control.py` must stay next to the `.xosc` files.
