# Munich Route_1

| | |
|---|---|
| CARLA map | `Munich_Route_1`, a 13 km corridor through Munich (OpenDRIVE with a tmerc `geoReference`) |
| Server | CARLA 0.9.16 server **v5**, `Munich_Route_1_carla0916_server_v5.tar.gz`: Hugging Face `Yuan-avs/Munich_Route_1_carla0916` (private), or the lab NAS (`AVS-Share`) `Yuan/Mifcom2/Maps/Munich/` |
| Scenarios | 117 scenarios: 39 each with light, moderate and dense traffic, 20 s at 10 Hz |
| Variants | one per munich2drive Route_1 camera clip (EDGAR as the ego, up to 10 per scenario) |
| Scenario files | folder `Munich_Route_1/` of the private Hugging Face dataset repo `<scenario dataset>`, or the lab NAS `Yuan/Mifcom2/Dataset/munich2drive_route1_odd/openscenario/` |

## Install and start

```bash
python scripts/download_server.py munich --dest ~/carla_servers
scripts/start_carla.sh ~/carla_servers/Munich_Route_1_carla0916_server_v5 4100
```

`download_server.py` also puts `german_light_heads.json` into the server folder, and
`start_carla.sh` prints the lamp arguments for `run_osc.py`. The Hugging Face repo also has its
own `install_route1.sh` and README with the server's history and checks.

## Run a variant

The scenario files: `~/m2d_osc/Munich_Route_1/` downloaded as in [setup.md](setup.md) step 4.

```bash
SRV=~/carla_servers/Munich_Route_1_carla0916_server_v5
python scripts/run_osc.py ~/m2d_osc/Munich_Route_1/variants/scenario_00_ego_6938.xosc --port 4100 --out runs/ \
    --sr-root ~/scenario_runner --carla-pythonapi $SRV/PythonAPI/carla \
    --lamp-bridge $SRV/lights_bridge/4100/control --lamp-heads $SRV/german_light_heads.json
```

## What is specific to this map

- **EDGAR:** the ego, TUM's research van, is spawned as `vehicle.lincoln.mkz_2017`; server v5
  carries EDGAR's mesh under that id. The van is 4.96 × 2.23 × 2.09 m. A stock CARLA shows a
  Lincoln MKZ instead (4.90 × 2.13 × 1.51 m).
- **German traffic lights:** every lamp of the map's 640 German signal heads exists as an
  on and an off mesh, so without control all of them look lit. Server v5 can switch them
  through a file bridge (`-GermanLightControl`). `run_osc.py --lamp-bridge` switches all heads
  dark before t = 0, as in the clips. In the clips, the heads at signalised junctions then
  followed the signal plan; the re-runs leave them dark.
- **Vehicles:** the European CARLA fleet, and Munich buses `vehicle.generic.citaro_12m` and
  `vehicle.generic.tourismo_12m`, which stock CARLA does not have (both munich2drive servers do).
- **Elevation:** the corridor lies at ~500–530 m with stacked roads; z in the files is the road
  surface (deck resolved).
