#!/usr/bin/env python3
"""Download, verify and unpack a CARLA 0.9.16 server with a munich2drive map, from Hugging Face or
from a copy you already have (e.g. on the lab NAS).

    python scripts/download_server.py munich  --dest ~/carla_servers
    python scripts/download_server.py bayarea --dest ~/carla_servers
    python scripts/download_server.py bayarea --dest ~/carla_servers \
        --archive <NAS mount>/Yuan/Mifcom2/Maps/California/BayArea_carla0916_server_v1.tar.gz

Both Hugging Face repos are PRIVATE: log in first (`hf auth login`, or set HF_TOKEN) with an
account that has been given access. The archive is checked against the SHA-256 file stored next
to it in the repo before anything is unpacked, and the unpacked server is checked for the files a
run needs. It refuses to unpack over an existing folder.

--archive PATH needs no Hugging Face login: the archive is checked against its known SHA-256
(SERVERS below), and german_light_heads.json is taken from the archive's folder. The TUM AVS lab
NAS (share AVS-Share) holds both archives with their .sha256 files:
  Yuan/Mifcom2/Maps/Munich/Munich_Route_1_carla0916_server_v5.tar.gz   (+ german_light_heads.json)
  Yuan/Mifcom2/Maps/California/BayArea_carla0916_server_v1.tar.gz

  munich   Yuan-avs/Munich_Route_1_carla0916  Munich_Route_1_carla0916_server_v5.tar.gz  15.8 GB -> ~39 GB
  bayarea  TUM-AVS/BayArea_carla0916          BayArea_carla0916_server_v1.tar.gz         20.0 GB -> ~51 GB
           (falls back to Yuan-avs/BayArea_carla0916, the original copy, when the TUM-AVS repo
           is not readable with your login)

--archive-dir keeps the downloaded archive (default: <dest>/_downloads); --keep-archive keeps it
after unpacking. An archive given with --archive is never deleted.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import subprocess
import sys
from pathlib import Path

SERVERS = {
    "munich": {
        "repos": ["Yuan-avs/Munich_Route_1_carla0916"],
        "archive": "Munich_Route_1_carla0916_server_v5.tar.gz",
        "sha256": "72f9f95bfea33273d065d02ae08a86862ab2492a80b6bb6a53c676d529f82176",
        "folder": "Munich_Route_1_carla0916_server_v5",
        "maps": ["Munich_Route_1"],
        "unpacked_gb": 39,
        # the German lamp heads of the map: run_osc.py --lamp-heads (switches them dark)
        "extras": ["german_light_heads.json"],
    },
    "bayarea": {
        "repos": ["TUM-AVS/BayArea_carla0916", "Yuan-avs/BayArea_carla0916"],
        "archive": "BayArea_carla0916_server_v1.tar.gz",
        "sha256": "e22fbe7cc9222329bdb96cfe7520afae0f68e96c146bab47ef4dc269668f9fcd",
        "folder": "BayArea_carla0916_server_v1",
        "maps": ["Page_Mill_Rd_Palo_Alto_CA", "Yale_St_Palo_Alto_CA",
                 "Belmont_Office_Park_Belmont_CA"],
        "unpacked_gb": 51,
    },
}


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(8 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch(repo: str, filename: str, out_dir: Path) -> Path:
    from huggingface_hub import hf_hub_download
    return Path(hf_hub_download(repo_id=repo, filename=filename, repo_type="dataset",
                                local_dir=str(out_dir)))


def verify_server(root: Path, maps: list[str]) -> list[str]:
    need = ["CarlaUE4.sh", "CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"]
    for m in maps:
        need += [f"CarlaUE4/Content/Carla/Maps/OpenDrive/{m}.xodr"]
    return [p for p in need if not (root / p).exists()]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("server", choices=sorted(SERVERS))
    ap.add_argument("--dest", type=Path, required=True, help="parent folder of the server")
    ap.add_argument("--archive-dir", type=Path, default=None)
    ap.add_argument("--archive", type=Path, default=None, help="use this local archive")
    ap.add_argument("--keep-archive", action="store_true")
    args = ap.parse_args()

    s = SERVERS[args.server]
    dest = args.dest.expanduser().resolve()
    target = dest / s["folder"]
    if target.exists():
        sys.exit(f"{target} exists already: remove it or choose another --dest")
    dest.mkdir(parents=True, exist_ok=True)

    adir = (args.archive_dir or dest / "_downloads").expanduser().resolve()
    adir.mkdir(parents=True, exist_ok=True)
    repo = None
    if args.archive:                        # a local copy: its known SHA-256, no login needed
        archive = args.archive.expanduser().resolve()
        if not archive.is_file():
            sys.exit(f"{archive} does not exist")
        want, archive_bytes = s["sha256"], 0
    else:
        from huggingface_hub import HfApi
        from huggingface_hub.utils import HfHubHTTPError, RepositoryNotFoundError
        sums = None
        for cand in s["repos"]:             # the first repo this login can read
            try:
                sums, repo = fetch(cand, s["archive"] + ".sha256", adir), cand
                break
            except (RepositoryNotFoundError, HfHubHTTPError) as e:
                print(f"  {cand}: not readable with this login ({type(e).__name__})")
        if sums is None:
            sys.exit(f"none of {s['repos']} is readable: log in with an account that has access, "
                     f"or pass --archive with a copy (e.g. from the lab NAS)")
        print(f"using {repo}")
        want = sums.read_text().split()[0].strip().lower()
        archive_bytes = next((f.size for f in HfApi().list_repo_tree(repo, repo_type="dataset")
                              if getattr(f, "path", "") == s["archive"]), 0)
    # the peak: the archive (unless it is already local) and the unpacked tree, plus 5 % each;
    # one budget when both live on one filesystem, else one per filesystem
    unpacked = s["unpacked_gb"] * 1e9 * 1.05
    archive_need = archive_bytes * 1.05
    if os.stat(adir).st_dev == os.stat(dest).st_dev:
        checks = [(dest, unpacked + archive_need)]
    else:
        checks = [(dest, unpacked), (adir, archive_need)]
    for where, need in checks:
        have = shutil.disk_usage(where).free
        if have < need:
            sys.exit(f"need ~{need / 1e9:.0f} GB free in {where}, have {have / 1e9:.0f} GB "
                     f"(archive {archive_bytes / 1e9:.0f} GB, unpacked {s['unpacked_gb']} GB)")
    if not args.archive:
        print(f"downloading {repo}/{s['archive']} -> {adir} ...", flush=True)
        archive = fetch(repo, s["archive"], adir)
    print(f"checking SHA-256 of {archive.name} ...", flush=True)
    got = sha256(archive)
    if got != want:
        sys.exit(f"SHA-256 MISMATCH for {archive}: {got} != {want} "
                 f"({'the known ' + s['archive'] if args.archive else 'repo ' + repo})")
    print(f"  OK {got}")

    print(f"unpacking into {dest} ...", flush=True)
    try:
        subprocess.run(["tar", "xzf", str(archive), "-C", str(dest)], check=True)
    except (subprocess.CalledProcessError, KeyboardInterrupt):
        shutil.rmtree(target, ignore_errors=True)       # a partial tree blocks the next attempt
        sys.exit(f"unpacking failed; removed the partial {target}")
    missing = verify_server(target, s["maps"])
    if missing:
        sys.exit(f"unpacked, but {target} lacks: {missing}")
    for extra in s.get("extras", []):
        local = archive.parent / extra if args.archive else None
        if local is not None and local.is_file():
            shutil.copyfile(local, target / extra)
            continue
        try:
            shutil.copyfile(fetch(repo or s["repos"][0], extra, adir), target / extra)
        except Exception as e:                  # noqa: BLE001 -- only the lamp switch needs it
            print(f"  note: no {extra} next to the archive or on Hugging Face "
                  f"({type(e).__name__}); run_osc.py --lamp-bridge needs it")
    if not args.keep_archive and not args.archive:
        archive.unlink()
    print(f"\nserver ready: {target}\n  maps: {', '.join(s['maps'])}\n"
          f"  start: scripts/start_carla.sh {target} 4100")


if __name__ == "__main__":
    main()
