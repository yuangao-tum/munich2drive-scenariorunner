# Bay Area (California)

| | |
|---|---|
| CARLA maps | `Page_Mill_Rd_Palo_Alto_CA` (63 km of road, 70 signals), `Yale_St_Palo_Alto_CA` (14 km, 46 signals), `Belmont_Office_Park_Belmont_CA` (13 km, no signals). RoadRunner exports, OpenDRIVE 1.4 with a tmerc `geoReference` |
| Server | CARLA 0.9.16 server **v1**, `BayArea_carla0916_server_v1.tar.gz`: Hugging Face `TUM-AVS/BayArea_carla0916` (private), or the lab NAS (`AVS-Share`) `Yuan/Mifcom2/Maps/California/` |
| Scenarios | 75 (Page Mill) + 63 (Yale) + 63 (Belmont) scenarios, 20 s at 10 Hz |
| Variants | 657 + 205 + 233 = 1,095 ego variants (EDGAR as the ego) |
| Scenario files | lab NAS (`AVS-Share`) `Yuan/Mifcom2/Dataset/bayarea_odd/data_bayarea_<pagemill, yale, belmont>/openscenario/`; four samples in `samples/` |

## Install and start

```bash
python scripts/download_server.py bayarea --dest ~/carla_servers
scripts/start_carla.sh ~/carla_servers/BayArea_carla0916_server_v1 4100
```

One server holds all three maps; ScenarioRunner loads the one a file names. Use this server only
for the Bay Area maps: it is a separate build, and 81 of its asset paths collide with Munich
content.

## Run a variant

The scenario files: `~/m2d_osc/<map>/` downloaded as in [setup.md](setup.md) step 4. Belmont files need `run_osc.py`: plain ScenarioRunner never finishes building its route planner on that map.

```bash
SRV=~/carla_servers/BayArea_carla0916_server_v1
python scripts/run_osc.py ~/m2d_osc/Page_Mill_Rd_Palo_Alto_CA/variants/scenario_00_ego_5105.xosc --port 4100 --out runs/ \
    --sr-root ~/scenario_runner --carla-pythonapi $SRV/PythonAPI/carla
```

## What is specific to these maps

- **Vehicles:** the US CARLA fleet (Charger, Mustang, Impala, Wrangler ... included; the yellow
  NYC taxi left out). Buses are the same `citaro_12m` / `tourismo_12m` models as in Munich.
- **Traffic lights:** in the clips, CARLA's light actors followed the scenarios' signal plans.
  The re-runs show CARLA's own cycles. Belmont has no signals.
- **Map edges:** the maps end abruptly; a camera looking out over the edge sees empty ground.
- **Ground height:** the RoadRunner export puts CARLA's spawn points 4 m above the road. The
  files carry the real road surface, so this does not matter for the re-runs.
