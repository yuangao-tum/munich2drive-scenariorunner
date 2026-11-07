#!/usr/bin/env python3
"""Compare a ScenarioRunner re-run with the original munich2drive clip: frames, PSNR and SSIM per view.

    python scripts/compare_clip.py runs/scenario_00_ego_5105 <clip folder> [--views front,back]

The clip folder holds one video per view whose name ends in _<view>.mp4; the re-run folder holds
<view>.mp4 (run_osc.py). Writes compare.json into the re-run folder. Both videos must have the
same size; frames are compared one to one (ffmpeg psnr/ssim).
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
from pathlib import Path

VIEWS = ("front", "front_left", "front_right", "back", "back_left", "back_right", "bird", "chase")


def n_frames(p: Path) -> int:
    out = subprocess.check_output(["ffprobe", "-v", "error", "-count_frames", "-select_streams",
                                   "v:0", "-show_entries", "stream=nb_read_frames", "-of",
                                   "csv=p=0", str(p)], text=True)
    return int(out.strip())


def metric(a: Path, b: Path, name: str) -> float | None:
    """ffmpeg's summary line: PSNR '... average:31.92 ...', SSIM '... All:0.912 (10.5)'."""
    p = subprocess.run(["ffmpeg", "-hide_banner", "-i", str(a), "-i", str(b), "-lavfi",
                        f"[0:v][1:v]{name}", "-f", "null", "-"], capture_output=True, text=True)
    pat = r"average:([0-9.]+|inf)" if name == "psnr" else r"All:([0-9.]+)"
    m = re.findall(pat, p.stderr)
    return float(m[-1]) if m and m[-1] != "inf" else (float("inf") if m else None)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("rerun", type=Path)
    ap.add_argument("original", type=Path)
    ap.add_argument("--views", default=",".join(VIEWS))
    args = ap.parse_args()
    clip = args.rerun.name
    res = {"clip": clip, "views": {}}
    for v in args.views.split(","):
        a = args.rerun / f"{v}.mp4"
        cands = sorted(args.original.glob(f"{clip}_*_{v}.mp4")) or sorted(
            args.original.glob(f"*_{v}.mp4"))
        cands = [c for c in cands if not c.name.endswith(f"_{v}_edge.mp4")]
        if not a.exists() or not cands:
            res["views"][v] = {"error": "missing video"}
            continue
        b = cands[0]
        res["views"][v] = {"original": b.name, "frames": [n_frames(a), n_frames(b)],
                           "psnr_db": metric(a, b, "psnr"), "ssim": metric(a, b, "ssim")}
        print(v, res["views"][v], flush=True)
    (args.rerun / "compare.json").write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
