# Sample scenarios

Seven munich2drive OpenSCENARIO files to try the runner with. They are the scenarios of the
[test report](../docs/test-report.md), unchanged from the published sets. Each one needs the CARLA
server of its map, from [docs/setup.md](../docs/setup.md) step 1.

| File | Server | Traffic | Weather | Agents | Refused at spawn, all recovered (test) | Run time (test) |
|---|---|---|---|---|---|---|
| `Munich_Route_1/scenario_00_ego_6938.xosc` | Munich v5 | light | WetCloudyNoon | 119 | 4 | 12.7 min |
| `Munich_Route_1/scenario_02_ego_7937.xosc` | Munich v5 | dense | WetCloudySunset | 189 | 0 | 7.9 min |
| `Munich_Route_1/scenario_02_ego_6944.xosc` | Munich v5 | dense | HardRainNoon | 83 | 48 | 11.4 min |
| `Page_Mill_Rd_Palo_Alto_CA/scenario_00_ego_5105.xosc` | Bay Area v1 | light | CloudySunset | 177 | 11 | 9.0 min |
| `Yale_St_Palo_Alto_CA/scenario_00_ego_1470.xosc` | Bay Area v1 | light | HardRainNoon | 186 | 9 | 4.4 min |
| `Belmont_Office_Park_Belmont_CA/scenario_00_ego_1585.xosc` | Bay Area v1 | light | ClearSunset | 138 | 6 | 7.5 min |
| `Belmont_Office_Park_Belmont_CA/scenario_00.xosc` (base, no ego) | Bay Area v1 | light | – | 138 | 6 | 0.4 min |

The run times are with all 16 videos, two servers sharing one RTX 5090.

A file named `scenario_XX_ego_YY.xosc` is an ego variant: vehicle YY of scenario XX drives as the
EDGAR ego (`hero`), and its camera views are recorded. `scenario_00.xosc` is a base scenario with
every agent and no ego, so there is nothing to film from: run it with `--no-record`.

## Run

```bash
# from the repository folder, with the Bay Area server running on port 4100
python scripts/run_osc.py samples/Page_Mill_Rd_Palo_Alto_CA/scenario_00_ego_5105.xosc \
    --port 4100 --out runs/ --sr-root ~/scenario_runner \
    --carla-pythonapi ~/carla-0.9.16-src/PythonAPI/carla

# the base scenario, without cameras
python scripts/run_osc.py samples/Belmont_Office_Park_Belmont_CA/scenario_00.xosc --no-record \
    --port 4100 --out runs/ --sr-root ~/scenario_runner \
    --carla-pythonapi ~/carla-0.9.16-src/PythonAPI/carla
```

On the Munich server, add the two `--lamp-bridge` / `--lamp-heads` arguments that `start_carla.sh`
prints. They switch the German traffic-light heads dark, as in the clips
([docs/munich.md](../docs/munich.md)).

The output and `run.json` are described in [docs/recording.md](../docs/recording.md). A run that
reproduces the scenario exits with 0.

## Plain ScenarioRunner

The `osc_replay_control.py` in each folder is a link to `../../osc/osc_replay_control.py`, the
controller the files assign. ScenarioRunner finds it next to the file. See
[docs/recording.md](../docs/recording.md), "Plain ScenarioRunner", for the command and its limits.

## More scenarios

The full sets (Munich 117 base / 1,043 variants; Bay Area 201 base / 1,095 variants) are private:
see [docs/setup.md](../docs/setup.md) step 4.
