# Setup

Linux with an NVIDIA GPU. One CARLA server needs about 8–9 GB of VRAM with the eight cameras
attached; two servers fit on a 24 GB card.

## 0. Environment

**Python 3.10 or 3.11.** ScenarioRunner 0.9.16 pins numpy 1.24.4, which has no wheel for 3.12.

- **Ubuntu 22.04:** Python 3.10 is the system Python; its venv module is the package
  `python3.10-venv`.
- **Ubuntu 24.04 and later:** these ship only 3.12. Get 3.10 from the deadsnakes PPA
  (`sudo add-apt-repository ppa:deadsnakes/ppa && sudo apt install python3.10 python3.10-venv`)
  or from conda, or let uv fetch it: replace the `python3.10 -m venv` line below with
  `uv venv --seed --python 3.10 ~/m2d-sr && . ~/m2d-sr/bin/activate` (`--seed` puts pip into the
  environment, which the `pip install` lines need).

```bash
sudo apt install ffmpeg git libvulkan1 tmux      # ffmpeg with libx264; CARLA renders through Vulkan
sudo apt install python3.10-venv                 # Ubuntu 22.04 (24.04: see above)
python3.10 -m venv ~/m2d-sr && . ~/m2d-sr/bin/activate
pip install -U pip "huggingface_hub>=0.34"
hf auth login                                    # or: export HF_TOKEN=...
```

**Driver.** The NVIDIA driver must support Vulkan; `vulkaninfo` (package `vulkan-tools`) checks it.

**Separate environment.** Use one for this repository only: ScenarioRunner pins
`antlr4-python3-runtime==4.10`, and a package built against ANTLR 4.9 (for example `omegaconf`)
cannot be imported next to it.

## 1. A CARLA server with the maps

The maps are not part of stock CARLA. Each map family ships as a complete, packaged CARLA 0.9.16
server, in a **private** Hugging Face dataset repo and on the TUM AVS lab NAS (share `AVS-Share`);
ask the owner for access.

| Map family | Hugging Face repo | Lab NAS (`AVS-Share`) | Archive | Unpacked |
|---|---|---|---|---|
| Munich Route_1 | `Yuan-avs/Munich_Route_1_carla0916` | `Yuan/Mifcom2/Maps/Munich/` | `Munich_Route_1_carla0916_server_v5.tar.gz` (15.8 GB) | ~39 GB |
| Bay Area (Page Mill, Yale, Belmont) | `TUM-AVS/BayArea_carla0916` | `Yuan/Mifcom2/Maps/California/` | `BayArea_carla0916_server_v1.tar.gz` (20.0 GB) | ~51 GB |

```bash
python scripts/download_server.py munich  --dest ~/carla_servers
python scripts/download_server.py bayarea --dest ~/carla_servers

# or from the NAS (no Hugging Face login needed), with the share mounted at /mnt/avs-share:
python scripts/download_server.py munich --dest ~/carla_servers \
    --archive /mnt/avs-share/Yuan/Mifcom2/Maps/Munich/Munich_Route_1_carla0916_server_v5.tar.gz
python scripts/download_server.py bayarea --dest ~/carla_servers \
    --archive /mnt/avs-share/Yuan/Mifcom2/Maps/California/BayArea_carla0916_server_v1.tar.gz
```

The script needs the archive and the unpacked tree on disk at the same time: about 57 GB free for
Munich and 74 GB for the Bay Area. It does the following:

- downloads the archive and checks it against the SHA-256 file stored next to it in the repo;
- unpacks it to `~/carla_servers/<server folder>/` and checks that the server binary and the maps'
  OpenDRIVE files are there;
- refuses to unpack over an existing folder, and removes a partial folder if unpacking fails;
- deletes the archive afterwards (`--keep-archive` keeps it);
- for Munich, also fetches `german_light_heads.json` into the server folder.

`--archive PATH` uses an archive you already have and needs no login: it is checked against the
archive's known SHA-256, never deleted, and for Munich `german_light_heads.json` is taken from the
archive's folder (the NAS folder has it). For the Bay Area the download tries
`TUM-AVS/BayArea_carla0916` first and falls back to `Yuan-avs/BayArea_carla0916`, the original copy.

Use each server only for its own maps. The Bay Area server is a separate build: 81 asset paths it
contains collide with Munich content, so never load Munich on it, or the reverse.

Start one of them from this repository's folder. `start_carla.sh` stays in the foreground until
the server exits, so run it in tmux (or screen); `tmux attach -t carla` shows its output, and
Ctrl-b d leaves it running:

```bash
tmux new -d -s carla scripts/start_carla.sh ~/carla_servers/Munich_Route_1_carla0916_server_v5 4100
# or
tmux new -d -s carla scripts/start_carla.sh ~/carla_servers/BayArea_carla0916_server_v1 4100
```

It uses the flags of the munich2drive clips: `-RenderOffScreen -quality-level=Epic -nosound`, RPC
port 4100. The server also uses 4101 and 4102. For a second server on the same machine, start it on
another port (e.g. 4110), and run `run_osc.py --port 4110` against it. The Traffic Manager port
defaults to the server port + 4000, so the two runs do not collide. On a machine with several GPUs
set `CARLA_GPU=<index>` (through tmux: `tmux new -d -s carla -e CARLA_GPU=<index> scripts/start_carla.sh ...`,
since a running tmux server does not pass on the caller's variables). On the Munich server the script also switches on the control of the map's
German traffic-light lamp heads, and prints the two `run_osc.py` arguments that use it.

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

ScenarioRunner also imports CARLA's `agents` package, which is not on PyPI. Take it from the
CARLA 0.9.16 source on GitHub; the sparse clone fetches only that folder (about 1 MB):

```bash
git clone --depth 1 --filter=blob:none --sparse --branch 0.9.16 \
    https://github.com/carla-simulator/carla.git ~/carla-0.9.16-src
git -C ~/carla-0.9.16-src sparse-checkout set PythonAPI/carla/agents
```

Then pass `--carla-pythonapi ~/carla-0.9.16-src/PythonAPI/carla` to `run_osc.py`. Each server
folder holds the same package, so `--carla-pythonapi ~/carla_servers/<server folder>/PythonAPI/carla`
or `CARLA_ROOT=~/carla_servers/<server folder>` work as well.

## 4. The scenario files

Seven scenarios to try are in [samples/](../samples/) (no download needed). The full OpenSCENARIO
sets are on the lab NAS (`AVS-Share`), in the `openscenario/` folder of each set:

| Set | Lab NAS (`AVS-Share`) |
|---|---|
| Munich_Route_1 | `Yuan/Mifcom2/Dataset/munich2drive_route1_odd/openscenario/` |
| Munich_Route_1, div1000 (2,000 scenarios) | `Yuan/Mifcom2/Dataset/munich2drive_div1000/openscenario/` |
| Page_Mill_Rd_Palo_Alto_CA | `Yuan/Mifcom2/Dataset/bayarea_odd/data_bayarea_pagemill/openscenario/` |
| Yale_St_Palo_Alto_CA | `Yuan/Mifcom2/Dataset/bayarea_odd/data_bayarea_yale/openscenario/` |
| Belmont_Office_Park_Belmont_CA | `Yuan/Mifcom2/Dataset/bayarea_odd/data_bayarea_belmont/openscenario/` |

```bash
# e.g. the Yale set, with the share mounted at /mnt/avs-share
cp -r /mnt/avs-share/Yuan/Mifcom2/Dataset/bayarea_odd/data_bayarea_yale/openscenario ~/m2d_osc/Yale_St_Palo_Alto_CA
```

Keep each set's folder as it is: `osc_replay_control.py` must stay next to the `.xosc` files.
`index.csv` lists the finished files of a set. The Munich sets also have `lamps/`: one lamp plan
per scenario, for `--lamp-plan` ([recording.md](recording.md#lamp-plans-munich)).
