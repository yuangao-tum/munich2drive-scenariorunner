#!/usr/bin/env bash
# Start a munich2drive CARLA 0.9.16 server headless, with the flags the clips were rendered with.
#
#   scripts/start_carla.sh <server_folder> [port]      # default port 4100 (uses port .. port+2)
#
# Run it in tmux/screen: it stays in the foreground until the server exits (Ctrl-C stops it).
# On a multi-GPU machine pick the card with CARLA_GPU=<index> (-> -graphicsadapter=<index>).
# The Munich server (v5) carries a GermanLightControl switch for its German lamp heads; it is
# turned on here with a file bridge under <server_folder>/lights_bridge/<port>/, and the
# run_osc.py arguments that use it are printed.
set -eu
SRV=$(cd "${1:?usage: start_carla.sh <server_folder> [port]}" && pwd)
PORT=${2:-4100}
[ -x "$SRV/CarlaUE4.sh" ] || { echo "no CarlaUE4.sh in $SRV" >&2; exit 1; }
ARGS=(-RenderOffScreen "-carla-rpc-port=$PORT" "-quality-level=${CARLA_QUALITY:-Epic}" -nosound)
[ -n "${CARLA_GPU:-}" ] && ARGS+=("-graphicsadapter=$CARLA_GPU")
BIN=$SRV/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping
# the switch is stored as UTF-16 in the binary: a plain grep never finds it. Both munich2drive
# servers carry it; only the Munich map has German lamp heads (german_light_heads.json, which
# download_server.py puts into the Munich server folder), so only then is it worth a hint.
if python3 -c "import sys; sys.exit(0 if 'GermanLightControl'.encode('utf-16-le') in open('$BIN','rb').read() else 1)"; then
    LB=$SRV/lights_bridge/$PORT
    mkdir -p "$LB"
    ARGS+=("-GermanLightControl=$LB/control")
    if [ -f "$SRV/german_light_heads.json" ]; then
        echo "German lamp control on. Pass to run_osc.py:"
        echo "    --lamp-bridge $LB/control --lamp-heads $SRV/german_light_heads.json"
    fi
fi
echo "starting $SRV on port $PORT: ${ARGS[*]}"
cd "$SRV"
exec ./CarlaUE4.sh "${ARGS[@]}"
