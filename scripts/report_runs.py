#!/usr/bin/env python3
"""Summarise run_osc.py runs (run.json, and compare.json when present) as a markdown table.

    python scripts/report_runs.py runs/scenario_00_ego_5105 runs/scenario_00_ego_6938 ...
"""
from __future__ import annotations

import json
import sys
from pathlib import Path


def mean(xs):
    xs = [x for x in xs if isinstance(x, (int, float))]
    return sum(xs) / len(xs) if xs else None


def row(d: Path) -> str:
    if not (d / "run.json").exists():                # the run died before writing it
        return f"| {d.name} | no run.json (see {d / 'scenario_runner.log'}) |" + " |" * 10
    r = json.loads((d / "run.json").read_text())
    c = json.loads((d / "compare.json").read_text()) if (d / "compare.json").exists() else None
    views = r.get("views") or {}
    frames = sorted({v["frames"] for v in views.values()}) if views else []
    missed = sum(v.get("missed", 0) for v in views.values())
    pa = r.get("pose_audit") or {}
    sp = r.get("spawn") or {}
    col = r.get("colors_as_file") or [0, 0]
    psnr = ssim = "–"
    if c:
        vv = [v for v in c["views"].values()
              if v.get("psnr_db") is not None and v.get("ssim") is not None]
        psnr = f"{mean([v['psnr_db'] for v in vv]):.1f}" if vv else "–"
        ssim = f"{mean([v['ssim'] for v in vv]):.3f}" if vv else "–"
    m = r.get("map", "")
    return (f"| {r['clip']} | {m} | {r['entities_in_file']} | {r['actors_in_run']} | "
            f"{sp.get('stock_refused', 0)} / {len(sp.get('respawned', {}))} / {len(sp.get('lost', []))} | "
            f"{col[0]}/{col[1]} | {'/'.join(map(str, frames)) or '–'} ({missed} missed) | "
            f"{pa.get('max_m', '–')} | {psnr} | {ssim} | {r['wall_s'] / 60:.1f} | "
            f"{'yes' if r.get('ok', r.get('completed')) else 'NO'} |")


def main():
    print("| Clip | Map | Entities | Actors | Refused / recovered / lost | Colours | Frames per view "
          "| Pose max (m) | PSNR (dB) | SSIM | Wall (min) | OK |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for a in sys.argv[1:]:
        print(row(Path(a)))


if __name__ == "__main__":
    main()
